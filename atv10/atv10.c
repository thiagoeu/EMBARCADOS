/* main.c
   Versão FreeRTOS: tarefas separadas para:
   - botões (ISR + fila)
   - buzzer (PWM)
   - LCD I2C
   - 7-segmentos
   - SD logger (leitura ADC + gravação)
*/

#include <stdio.h>
#include <string.h> 
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

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

static const char *TAG = "NTC_DAQ_S3_FREERTOS";

/* ----------------- Pin mapping ----------------- */
#define I2C_PORT I2C_NUM_0
#define SDA_PIN 16
#define SCL_PIN 17
#define LCD_ADDR 0x27

#define NTC_ADC_GPIO 4
#define NTC_ADC_CHANNEL ADC_CHANNEL_3

#define BUTTON_A GPIO_NUM_41
#define BUTTON_B GPIO_NUM_42

#define BUZZER_PIN GPIO_NUM_38
#define BUZZER_LEDC_TIMER LEDC_TIMER_0
#define BUZZER_LEDC_CHANNEL LEDC_CHANNEL_0
#define BUZZER_BITS 10
#define BUZZER_FREQ_HZ 2000

#define LED1_PIN 15
#define LED2_PIN 5
#define LED3_PIN 6
#define LED4_PIN 7

#define SEG_A GPIO_NUM_37
#define SEG_B GPIO_NUM_36
#define SEG_C GPIO_NUM_35
#define SEG_D GPIO_NUM_45
#define SEG_E GPIO_NUM_39
#define SEG_F GPIO_NUM_40
#define SEG_G GPIO_NUM_2

#define NTC_R_SERIE 10000.0f
#define NTC_BETA 3950.0f
#define NTC_R0 10000.0f
#define NTC_T0 298.15f /* 25°C in Kelvin */

#define DEBOUNCE_US 200000LL

#define SD_SPI_MOSI 47
#define SD_SPI_MISO 21
#define SD_SPI_SCLK 20
#define SD_SPI_CS   48
#define MOUNT_POINT "/sdcard"

typedef enum { EVT_BTN_A, EVT_BTN_B } button_evt_t;

static QueueHandle_t btn_evt_queue = NULL;
static SemaphoreHandle_t mutex_temp = NULL;
static volatile int temp_alarme = 25;
static volatile float temp_atual = 0.0f;
static volatile int64_t last_press_a = 0;
static volatile int64_t last_press_b = 0;
static adc_oneshot_unit_handle_t adc_handle = NULL;
static sdmmc_card_t *sd_card = NULL;

/* ---------- LCD helper ---------- */
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

/* ---------- ADC ---------- */
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

/* ---------- 7-segment ---------- */
static const gpio_num_t seg_gpio[7] = { SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F, SEG_G };
static const bool digit_map[][7] = {
    /*0*/ {1,1,1,1,1,1,0}, /*3*/ {1,1,1,1,0,0,1}, /*7*/ {1,1,1,0,0,0,0},
    /*D*/ {0,1,1,1,1,0,1}, /*F*/ {1,0,0,0,1,1,1}
};

static void sevenseg_init(void) {
    gpio_config_t io = { .mode = GPIO_MODE_OUTPUT, .pin_bit_mask = 0 };
    for (int i=0;i<7;i++) io.pin_bit_mask |= (1ULL<<seg_gpio[i]);
    gpio_config(&io);
}

static void sevenseg_write_char(char c) {
    const bool *map = NULL;
    switch(c) {
        case '0': map = digit_map[0]; break;
        case '3': map = digit_map[1]; break;
        case '7': map = digit_map[2]; break;
        case 'D': map = digit_map[3]; break;
        case 'F': map = digit_map[4]; break;
        default: for(int i=0;i<7;i++) gpio_set_level(seg_gpio[i],0); return;
    }
    for(int i=0;i<7;i++) gpio_set_level(seg_gpio[i], map[i]);
}

/* ---------- Buttons ISR ---------- */
static void IRAM_ATTR button_a_isr(void *arg) {
    int64_t now = esp_timer_get_time();
    if(now - last_press_a > DEBOUNCE_US){
        last_press_a = now;
        button_evt_t e = EVT_BTN_A;
        BaseType_t px = pdFALSE;
        xQueueSendFromISR(btn_evt_queue,&e,&px);
        if(px) portYIELD_FROM_ISR();
    }
}
static void IRAM_ATTR button_b_isr(void *arg) {
    int64_t now = esp_timer_get_time();
    if(now - last_press_b > DEBOUNCE_US){
        last_press_b = now;
        button_evt_t e = EVT_BTN_B;
        BaseType_t px = pdFALSE;
        xQueueSendFromISR(btn_evt_queue,&e,&px);
        if(px) portYIELD_FROM_ISR();
    }
}

static void buttons_init(void) {
    gpio_config_t io = {0};
    io.intr_type = GPIO_INTR_NEGEDGE;
    io.mode = GPIO_MODE_INPUT;
    io.pin_bit_mask = (1ULL<<BUTTON_A) | (1ULL<<BUTTON_B);
    io.pull_up_en = 1;
    gpio_config(&io);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_A, button_a_isr, NULL);
    gpio_isr_handler_add(BUTTON_B, button_b_isr, NULL);
}

/* ---------- ADC init ---------- */
static void adc_init_for_ntc(void){
    adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg,&adc_handle));
    adc_oneshot_chan_cfg_t ch_cfg = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle,NTC_ADC_CHANNEL,&ch_cfg));
}

/* ---------- SD mount ---------- */
static sdmmc_card_t* mount_sdcard(void){
    esp_err_t ret;
    sdmmc_card_t *card = NULL;
    esp_vfs_fat_sdmmc_mount_config_t cfg = {.format_if_mount_failed=false, .max_files=5, .allocation_unit_size=16*1024};
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
    .mosi_io_num = SD_SPI_MOSI,
    .miso_io_num = SD_SPI_MISO,
    .sclk_io_num = SD_SPI_SCLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4000,
    .data4_io_num = -1,
    .data5_io_num = -1,
    .data6_io_num = -1,
    .data7_io_num = -1
};
    ret = spi_bus_initialize(host.slot,&bus_cfg,SPI_DMA_CH_AUTO);
    if(ret!=ESP_OK){ESP_LOGE(TAG,"SPI init fail %s",esp_err_to_name(ret));return NULL;}
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_SPI_CS;
    slot_config.host_id = host.slot;
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT,&host,&slot_config,&cfg,&card);
    if(ret!=ESP_OK){ESP_LOGE(TAG,"Failed to mount SD %s",esp_err_to_name(ret));spi_bus_free(host.slot);return NULL;}
    sdmmc_card_print_info(stdout,card);
    ESP_LOGI(TAG,"SD mounted to %s",MOUNT_POINT);
    return card;
}

/* ---------- Tasks ---------- */

/* Buttons -> update alarm */
static void task_buttons(void *pv){
    button_evt_t evt;
    for(;;){
        if(xQueueReceive(btn_evt_queue,&evt,portMAX_DELAY)==pdTRUE){
            xSemaphoreTake(mutex_temp,portMAX_DELAY);
            if(evt==EVT_BTN_A){ temp_alarme+=5; if(temp_alarme>100) temp_alarme=100;}
            else { temp_alarme-=5; if(temp_alarme<-40) temp_alarme=-40;}
            xSemaphoreGive(mutex_temp);
        }
    }
}

/* SD Logger */
static void task_sd_logger(void *pv){
    const TickType_t period = pdMS_TO_TICKS(5000);
    TickType_t last_wake = xTaskGetTickCount();
    for(;;){
        vTaskDelayUntil(&last_wake,period);
        int raw = adc_read_avg(10);
        float t = ntc_raw_to_celsius(raw);
        xSemaphoreTake(mutex_temp,portMAX_DELAY); temp_atual=t; xSemaphoreGive(mutex_temp);
        if(sd_card){
            char path[128]; snprintf(path,sizeof(path), MOUNT_POINT"/ntc_log.csv");
            FILE *f = fopen(path,"a");
            if(f){ fprintf(f,"%lld,%d,%.2f,%d\n",(long long)esp_timer_get_time(),raw,t,temp_alarme); fclose(f);}
        }
    }
}

/* LCD */
static void task_lcd_display(void *pv){
    char line[17]; float last_temp=-9999; int last_alarm=9999;
    for(;;){
        xSemaphoreTake(mutex_temp,portMAX_DELAY);
        float t=temp_atual; int alarm=temp_alarme;
        xSemaphoreGive(mutex_temp);
        if(fabsf(t-last_temp)>0.2f || alarm!=last_alarm){
            lcd_set_cursor(0,0); snprintf(line,sizeof(line),"Temp:%5.1f C   ",t); lcd_print(line);
            lcd_set_cursor(0,1); snprintf(line,sizeof(line),"Alarme:%3d C    ",alarm); lcd_print(line);
            last_temp=t; last_alarm=alarm;
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

/* Buzzer + LEDs */
static void task_buzzer_pwm(void *pv){
    bool buzzer_enabled=false; bool blink_state=false;
    for(;;){
        xSemaphoreTake(mutex_temp,portMAX_DELAY);
        float t=temp_atual; int alarm=temp_alarme;
        xSemaphoreGive(mutex_temp);
        float diff = alarm - t;
        if(t>=alarm){ if(!buzzer_enabled){buzzer_on();buzzer_enabled=true;}
            blink_state = !blink_state; set_all_leds(blink_state,blink_state,blink_state,blink_state);}
        else{
            if(buzzer_enabled){buzzer_off();buzzer_enabled=false;}
            if(diff<=2.0f) set_all_leds(1,1,1,1);
            else if(diff<=10.0f) set_all_leds(1,1,1,0);
            else if(diff<=15.0f) set_all_leds(1,1,0,0);
            else if(diff<=20.0f) set_all_leds(1,0,0,0);
            else set_all_leds(0,0,0,0);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* Seven-segment */
static void task_sevenseg(void *pv){
    bool blink_on=false;
    for(;;){
        xSemaphoreTake(mutex_temp,portMAX_DELAY);
        float t=temp_atual; int alarm=temp_alarme;
        xSemaphoreGive(mutex_temp);
        float diff = alarm - t;
        if(t>=alarm){ blink_on=!blink_on; if(blink_on) sevenseg_write_char('F'); else sevenseg_write_char(0);}
        else if(diff<=2.0f) sevenseg_write_char('D');
        else if(diff<=10.0f) sevenseg_write_char('7');
        else if(diff<=15.0f) sevenseg_write_char('3');
        else if(diff<=20.0f) sevenseg_write_char('0');
        else sevenseg_write_char(0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ---------- app_main ---------- */
void app_main(void) {
    ESP_LOGI(TAG, "Inicializando periféricos...");

    /* Inicializa LCD */
    lcd_init();
    lcd_set_cursor(0, 0);
    lcd_print("Inicializando...");
    vTaskDelay(pdMS_TO_TICKS(200));

    /* Inicializa ADC para NTC */
    adc_init_for_ntc();

    /* Inicializa buzzer, LEDs e 7-segmentos */
    buzzer_init();
    leds_init();
    sevenseg_init();

    /* Cria mutex para temperatura */
    mutex_temp = xSemaphoreCreateMutex();
    if (mutex_temp == NULL) {
        ESP_LOGE(TAG, "Falha ao criar mutex_temp");
        abort();
    }

    /* Cria fila de eventos dos botões */
    btn_evt_queue = xQueueCreate(8, sizeof(button_evt_t));
    if (btn_evt_queue == NULL) {
        ESP_LOGE(TAG, "Falha ao criar btn_evt_queue");
        abort();
    }

    /* Inicializa botões com ISR */
    buttons_init();

    /* Monta SD card */
    sd_card = mount_sdcard();
    if (sd_card == NULL) {
        ESP_LOGW(TAG, "SD não montado - gravações serão ignoradas");
    }

    /* Cria tasks */
    xTaskCreate(task_buttons, "Buttons", 2048, NULL, 8, NULL);
    xTaskCreate(task_sd_logger, "SDLogger", 4096, NULL, 5, NULL);
    xTaskCreate(task_lcd_display, "LCD", 3072, NULL, 4, NULL);
    xTaskCreate(task_buzzer_pwm, "Buzzer", 2048, NULL, 6, NULL);
    xTaskCreate(task_sevenseg, "7SEG", 2048, NULL, 3, NULL);

    ESP_LOGI(TAG, "Tasks criadas. Sistema em execução.");
}
