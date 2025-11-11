/* main.c
   NTC + máquina de estado + SD (SDSPI) + LCD I2C + LEDs + Buzzer
   Adaptado para microSD via SPI usando pinos do seu JSON Wokwi:
   SCLK = GPIO20, MISO = GPIO21, MOSI = GPIO47, CS = GPIO48
*/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"

/* ADC moderno (oneshot) */
#include "esp_adc/adc_oneshot.h"

/* SD via SDSPI */
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "ff.h"

static const char *TAG = "NTC_DAQ_S3";

/* ----------------- Pin mapping ----------------- */
#define I2C_PORT I2C_NUM_0
#define SDA_PIN 16
#define SCL_PIN 17
#define LCD_ADDR 0x27

/* ADC (NTC) — usando ADC1_CH3 (GPIO4) */
#define NTC_ADC_GPIO 4
#define NTC_ADC_CHANNEL ADC_CHANNEL_3

/* Botões (interrupção) */
#define BUTTON_A GPIO_NUM_41
#define BUTTON_B GPIO_NUM_42

/* Buzzer (PWM) */
#define BUZZER_PIN GPIO_NUM_38
#define BUZZER_LEDC_TIMER LEDC_TIMER_0
#define BUZZER_LEDC_CHANNEL LEDC_CHANNEL_0
#define BUZZER_BITS 10
#define BUZZER_FREQ_HZ 2000

/* LEDs */
#define LED1_PIN 15
#define LED2_PIN 5
#define LED3_PIN 6
#define LED4_PIN 7

/* NTC constants */
#define NTC_R_SERIE 10000.0f
#define NTC_BETA 3950.0f
#define NTC_R0 10000.0f
#define NTC_T0 298.15f /* 25°C in Kelvin */

/* Debounce */
#define DEBOUNCE_US 200000LL /* 200 ms */

/* SD (SDSPI) pinos conforme JSON Wokwi */
#define SD_SPI_MOSI 47
#define SD_SPI_MISO 21
#define SD_SPI_SCLK 20
#define SD_SPI_CS   48

/* SD mount point */
#define MOUNT_POINT "/sdcard"

/* Queue for button events from ISRs */
typedef enum {
    EVT_BTN_A,
    EVT_BTN_B
} button_evt_t;

static QueueHandle_t btn_evt_queue = NULL;

/* Alarm temp stored as integer degrees Celsius for atomicity in ISRs/tasks */
static volatile int temp_alarme = 25;

/* For debounce timing inside ISRs */
static volatile int64_t last_press_a = 0;
static volatile int64_t last_press_b = 0;

/* ADC handle */
static adc_oneshot_unit_handle_t adc_handle = NULL;

/* ---------- LCD (PCF8574) helper ---------- */
#define LCD_RS  0x01
#define LCD_RW  0x02
#define LCD_EN  0x04
#define LCD_BL  0x08

static esp_err_t i2c_write_byte(uint8_t data) {
    return i2c_master_write_to_device(I2C_PORT, LCD_ADDR, &data, 1, pdMS_TO_TICKS(100));
}
static void lcd_pulse(uint8_t data) {
    i2c_write_byte(data | LCD_EN);
    esp_rom_delay_us(1);
    i2c_write_byte(data & ~LCD_EN);
    esp_rom_delay_us(50);
}
static void lcd_write_nibble(uint8_t nibble, uint8_t mode) {
    uint8_t data = (nibble << 4) | mode | LCD_BL;
    lcd_pulse(data);
}
static void lcd_send_cmd(uint8_t cmd) {
    lcd_write_nibble(cmd >> 4, 0);
    lcd_write_nibble(cmd & 0x0F, 0);
}
static void lcd_send_data(uint8_t d) {
    lcd_write_nibble(d >> 4, LCD_RS);
    lcd_write_nibble(d & 0x0F, LCD_RS);
}
static void lcd_init(void) {
    i2c_config_t conf = {0};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = SDA_PIN;
    conf.scl_io_num = SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_write_nibble(0x03, 0); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write_nibble(0x03, 0); vTaskDelay(pdMS_TO_TICKS(1));
    lcd_write_nibble(0x03, 0); vTaskDelay(pdMS_TO_TICKS(1));
    lcd_write_nibble(0x02, 0); vTaskDelay(pdMS_TO_TICKS(1));
    lcd_send_cmd(0x28);
    lcd_send_cmd(0x0C);
    lcd_send_cmd(0x06);
    lcd_send_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(2));
}
static void lcd_set_cursor(uint8_t col, uint8_t row) {
    uint8_t addr = (row == 0 ? 0x00 : 0x40) + col;
    lcd_send_cmd(0x80 | addr);
}
static void lcd_print(const char *s) {
    while (*s) lcd_send_data(*s++);
}

/* ---------- ADC helper ---------- */
static int adc_read_avg(int samples) {
    int sum = 0, raw = 0;
    for (int i = 0; i < samples; i++) {
        adc_oneshot_read(adc_handle, NTC_ADC_CHANNEL, &raw);
        sum += raw;
        esp_rom_delay_us(500);
    }
    return sum / samples;
}
static float ntc_raw_to_celsius(int raw) {
    if (raw <= 0) return -999.0f;
    float v = (raw / 4095.0f) * 3.3f;
    float r_ntc = (NTC_R_SERIE * v) / (3.3f - v);
    float tempK = 1.0f / ((1.0f / NTC_T0) + (1.0f / NTC_BETA) * logf(r_ntc / NTC_R0));
    return tempK - 273.15f;
}

/* ---------- Buzzer ---------- */
static void buzzer_init(void) {
    ledc_timer_config_t t = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = BUZZER_BITS,
        .timer_num = BUZZER_LEDC_TIMER,
        .freq_hz = BUZZER_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&t);
    ledc_channel_config_t ch = {
        .channel = BUZZER_LEDC_CHANNEL,
        .duty = 0,
        .gpio_num = BUZZER_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = BUZZER_LEDC_TIMER
    };
    ledc_channel_config(&ch);
}
static void buzzer_on(void) {
    uint32_t max = (1 << BUZZER_BITS) - 1;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL, max / 2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL);
}
static void buzzer_off(void) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL);
}

/* ---------- LEDs ---------- */
static void leds_init(void) {
    gpio_config_t io = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL<<LED1_PIN) | (1ULL<<LED2_PIN) |
                        (1ULL<<LED3_PIN) | (1ULL<<LED4_PIN),
    };
    gpio_config(&io);
}
static void set_all_leds(int a,int b,int c,int d){
    gpio_set_level(LED1_PIN, a);
    gpio_set_level(LED2_PIN, b);
    gpio_set_level(LED3_PIN, c);
    gpio_set_level(LED4_PIN, d);
}

/* ---------- Buttons ISR (very light) ---------- */
static void IRAM_ATTR button_a_isr(void *arg) {
    int64_t now = esp_timer_get_time();
    if (now - last_press_a > DEBOUNCE_US) {
        last_press_a = now;
        button_evt_t e = EVT_BTN_A;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(btn_evt_queue, &e, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
    }
}
static void IRAM_ATTR button_b_isr(void *arg) {
    int64_t now = esp_timer_get_time();
    if (now - last_press_b > DEBOUNCE_US) {
        last_press_b = now;
        button_evt_t e = EVT_BTN_B;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(btn_evt_queue, &e, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
    }
}
static void buttons_init(void) {
    gpio_config_t io = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL<<BUTTON_A) | (1ULL<<BUTTON_B),
        .pull_up_en = 1
    };
    gpio_config(&io);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_A, button_a_isr, NULL);
    gpio_isr_handler_add(BUTTON_B, button_b_isr, NULL);
}

/* ---------- ADC init ---------- */
static void adc_init_for_ntc(void) {
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));
    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten = ADC_ATTEN_DB_12,           // corrigido (não-deprecated)
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, NTC_ADC_CHANNEL, &ch_cfg));
}

/* ---------- SD Card mount (SDSPI) ---------- */
static sdmmc_card_t* mount_sdcard(void) {
    esp_err_t ret;
    sdmmc_card_t* card = NULL;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    /* host para SDSPI */
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    /* configurar barramento SPI (MOSi, MISO, SCLK) -> usar host.slot como SPI host */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_SPI_MOSI,
        .miso_io_num = SD_SPI_MISO,
        .sclk_io_num = SD_SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000
    };

    /* inicializa o barramento SPI (escolhemos o host definido pelo SDSPI_HOST_DEFAULT) */
    ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return NULL;
    }

    /* configura o dispositivo SDSPI (CS) */
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_SPI_CS;
    slot_config.host_id = host.slot;

    ESP_LOGI(TAG, "Mounting SD card (SDSPI)...");
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD (esp_vfs_fat_sdspi_mount): %s", esp_err_to_name(ret));
        /* Se falhar, desfaz o barramento SPI */
        spi_bus_free(host.slot);
        return NULL;
    }

    sdmmc_card_print_info(stdout, card);
    ESP_LOGI(TAG, "SD mounted to %s", MOUNT_POINT);
    return card;
}

/* ---------- Machine states ---------- */
typedef enum {
    STATE_INIT,
    STATE_READ,
    STATE_UPDATE_UI,
    STATE_ALARM,
    STATE_SAVE
} app_state_t;

/* ---------- Main task: state machine ---------- */
void app_main(void) {
    ESP_LOGI(TAG, "Inicializando periféricos...");
    lcd_init();
    lcd_set_cursor(0,0);
    lcd_print("Inicializando...");
    vTaskDelay(pdMS_TO_TICKS(500));

    /* init hw */
    adc_init_for_ntc();
    buzzer_init();
    leds_init();

    btn_evt_queue = xQueueCreate(8, sizeof(button_evt_t));
    buttons_init();

    sdmmc_card_t *card = mount_sdcard(); // pode ser NULL (tratar)

    float temp_atual = 0.0f;
    float last_display_temp = -9999;
    int last_display_alarm = -9999;
    int blink_state = 0;
    bool buzzer_enabled = false;
    char line[17];

    app_state_t state = STATE_READ;
    TickType_t last_save_tick = xTaskGetTickCount();

    for (;;) {
        switch (state) {
            case STATE_READ: {
                /* lidar com eventos dos botões (fila) */
                button_evt_t evt;
                while (xQueueReceive(btn_evt_queue, &evt, 0) == pdTRUE) {
                    if (evt == EVT_BTN_A) {
                        temp_alarme += 5;
                        if (temp_alarme > 100) temp_alarme = 100;
                        ESP_LOGI(TAG, "Alarme incrementado: %d", temp_alarme);
                    } else if (evt == EVT_BTN_B) {
                        temp_alarme -= 5;
                        if (temp_alarme < -40) temp_alarme = -40;
                        ESP_LOGI(TAG, "Alarme decrementado: %d", temp_alarme);
                    }
                }

                int raw = adc_read_avg(10);
                temp_atual = ntc_raw_to_celsius(raw);
                ESP_LOGI(TAG, "ADC raw(avg)=%d -> Temp=%.2f C (alarme %d)", raw, temp_atual, temp_alarme);

                /* salvar leitura periodicamente (por exemplo, a cada 5s) */
                if ((xTaskGetTickCount() - last_save_tick) > pdMS_TO_TICKS(5000)) {
                    state = STATE_SAVE;
                } else {
                    state = STATE_UPDATE_UI;
                }
                break;
            }

            case STATE_UPDATE_UI: {
                /* Atualiza LCD apenas se mudou */
                if (fabsf(temp_atual - last_display_temp) > 0.2f || temp_alarme != last_display_alarm) {
                    lcd_set_cursor(0,0);
                    snprintf(line, sizeof(line), "Temp:%5.1f C   ", temp_atual);
                    lcd_print(line);
                    lcd_set_cursor(0,1);
                    snprintf(line, sizeof(line), "Alarme:%3d C    ", temp_alarme);
                    lcd_print(line);
                    last_display_temp = temp_atual;
                    last_display_alarm = temp_alarme;
                }

                /* LEDs e buzzer */
                float diff = (float)temp_alarme - temp_atual;
                if (temp_atual >= temp_alarme) {
                    /* ativar alarme sonoro + piscar LEDs */
                    if (!buzzer_enabled) {
                        buzzer_on();
                        buzzer_enabled = true;
                    }
                    blink_state = !blink_state;
                    set_all_leds(blink_state, blink_state, blink_state, blink_state);
                } else {
                    if (buzzer_enabled) {
                        buzzer_off();
                        buzzer_enabled = false;
                    }
                    if (diff <= 2.0f) set_all_leds(1,1,1,1);
                    else if (diff <= 10.0f) set_all_leds(1,1,1,0);
                    else if (diff <= 15.0f) set_all_leds(1,1,0,0);
                    else if (diff <= 20.0f) set_all_leds(1,0,0,0);
                    else set_all_leds(0,0,0,0);
                }

                vTaskDelay(pdMS_TO_TICKS(300)); // refresh rate UI
                state = STATE_READ;
                break;
            }

            case STATE_SAVE: {
                /* Salva a leitura raw e temperatura no SD (CSV) */
                if (card != NULL) {
                    char path[128];
                    int raw = adc_read_avg(1); // já temos temp_atual mas guardamos raw também
                    snprintf(path, sizeof(path), MOUNT_POINT"/ntc_log.csv");
                    FILE *f = fopen(path, "a");
                    if (f != NULL) {
                        int64_t us = esp_timer_get_time();
                        /* formato: timestamp_us,raw,temp_c,alarm */
                        fprintf(f, "%lld,%d,%.2f,%d\n", (long long)us, raw, temp_atual, temp_alarme);
                        fclose(f);
                        ESP_LOGI(TAG, "Salvo no SD: %s", path);
                    } else {
                        ESP_LOGE(TAG, "Falha ao abrir %s para escrita", path);
                    }
                } else {
                    ESP_LOGW(TAG, "SD não montado: pulando gravação");
                }
                last_save_tick = xTaskGetTickCount();
                state = STATE_UPDATE_UI;
                break;
            }

            default:
                state = STATE_READ;
                break;
        }
    }
}
