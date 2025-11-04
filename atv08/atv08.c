#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "rom/ets_sys.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_err.h"

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
#define LED1_PIN GPIO_NUM_15
#define LED2_PIN GPIO_NUM_5
#define LED3_PIN GPIO_NUM_6
#define LED4_PIN GPIO_NUM_7

/* NTC constants */
#define NTC_R_SERIE 10000.0f
#define NTC_BETA 3950.0f
#define NTC_R0 10000.0f
#define NTC_T0 298.15f /* 25°C in Kelvin */

/* Debounce */
#define DEBOUNCE_US 200000LL /* 200 ms */

static volatile float temp_alarme = 25.0f;
static volatile int64_t last_press_a = 0;
static volatile int64_t last_press_b = 0;

/* ADC handle */
static adc_oneshot_unit_handle_t adc_handle = NULL;

/* ---- LCD (I2C PCF8574) ---- */
#define LCD_RS  0x01
#define LCD_RW  0x02
#define LCD_EN  0x04
#define LCD_BL  0x08

static esp_err_t i2c_write_byte(uint8_t data) {
    return i2c_master_write_to_device(I2C_PORT, LCD_ADDR, &data, 1, pdMS_TO_TICKS(100));
}
static void lcd_pulse(uint8_t data) {
    i2c_write_byte(data | LCD_EN);
    ets_delay_us(1);
    i2c_write_byte(data & ~LCD_EN);
    ets_delay_us(50);
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

/* ---------- Filtro ADC e conversão ---------- */
static int adc_read_avg(int samples) {
    int sum = 0, raw = 0;
    for (int i = 0; i < samples; i++) {
        adc_oneshot_read(adc_handle, NTC_ADC_CHANNEL, &raw);
        sum += raw;
        ets_delay_us(500); // pequeno delay para estabilidade
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

/* ---------- Botões ---------- */
static void IRAM_ATTR button_a_isr(void *arg) {
    int64_t now = esp_timer_get_time();
    if (now - last_press_a > DEBOUNCE_US) {
        last_press_a = now;
        temp_alarme += 5.0f;
        if (temp_alarme > 100.0f) temp_alarme = 100.0f;
    }
}
static void IRAM_ATTR button_b_isr(void *arg) {
    int64_t now = esp_timer_get_time();
    if (now - last_press_b > DEBOUNCE_US) {
        last_press_b = now;
        temp_alarme -= 5.0f;
        if (temp_alarme < -40.0f) temp_alarme = -40.0f;
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
    adc_oneshot_new_unit(&init_cfg, &adc_handle);
    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    adc_oneshot_config_channel(adc_handle, NTC_ADC_CHANNEL, &ch_cfg);
}

/* ---------- Main ---------- */
void app_main(void) {
    ESP_LOGI(TAG, "Inicializando periféricos...");
    lcd_init();
    lcd_set_cursor(0,0);
    lcd_print("Inicializando...");
    vTaskDelay(pdMS_TO_TICKS(500));

    adc_init_for_ntc();
    buzzer_init();
    leds_init();
    buttons_init();

    float temp_atual = 0.0f;
    float last_display_temp = -9999;
    float last_display_alarm = -9999;
    int blink_state = 0;
    bool buzzer_enabled = false;
    char line[17];

    while (1) {
        int raw = adc_read_avg(10);  // <-- média de 10 leituras
        temp_atual = ntc_raw_to_celsius(raw);

        ESP_LOGI(TAG, "ADC raw(avg)=%d -> Temp=%.2f C (alarme %.1f)", raw, temp_atual, temp_alarme);

        if (fabsf(temp_atual - last_display_temp) > 0.2f ||
            fabsf(temp_alarme - last_display_alarm) > 0.2f) {
            lcd_set_cursor(0,0);
            snprintf(line, sizeof(line), "Temp: %5.1f C", temp_atual);
            lcd_print("                ");
            lcd_set_cursor(0,0);
            lcd_print(line);
            lcd_set_cursor(0,1);
            snprintf(line, sizeof(line), "Alarme:%5.1fC", temp_alarme);
            lcd_print("                ");
            lcd_set_cursor(0,1);
            lcd_print(line);
            last_display_temp = temp_atual;
            last_display_alarm = temp_alarme;
        }

        float diff = temp_alarme - temp_atual;
        if (temp_atual >= temp_alarme) {
            buzzer_on();
            buzzer_enabled = true;
            blink_state = !blink_state;
            set_all_leds(blink_state, blink_state, blink_state, blink_state);
        } else {
            if (buzzer_enabled) {
                buzzer_off();
                buzzer_enabled = false;
            }
            if (diff <= 2) set_all_leds(1,1,1,1);
            else if (diff <= 10) set_all_leds(1,1,1,0);
            else if (diff <= 15) set_all_leds(1,1,0,0);
            else if (diff <= 20) set_all_leds(1,0,0,0);
            else set_all_leds(0,0,0,0);
        }

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}
