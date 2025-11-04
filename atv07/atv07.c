#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

// -------------------------------------------------------------------
// ----- CONFIGURAÇÕES DE HARDWARE E CONSTANTES (PINOS MANTIDOS) -----
// -------------------------------------------------------------------
#define I2C_MASTER_SCL_IO 20 // SCL (Clock) - PINO ORIGINAL MANTIDO
#define I2C_MASTER_SDA_IO 21 // SDA (Data)  - PINO ORIGINAL MANTIDO
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define MPU6050_ADDR 0x68
#define SSD1306_ADDR 0x3C
#define LED_GPIO 7           // PINO ORIGINAL MANTIDO

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_BUF_SIZE (SCREEN_WIDTH*SCREEN_HEIGHT/8)

// Constantes para MPU6050 (Aceleração)
#define MPU_SCALE_FACTOR 16384.0 // LSB/g para faixa de +/- 2g (padrão)
#define G_TO_MS2 9.81            // Fator de conversão de g para m/s^2
#define DELTA_MS2_LIMITE 0.5     // Limite de 0.5 m/s² para acender o LED
#define INTERVALO_MS 200         // Intervalo de leitura (200 ms)
#define NUM_MEDIDAS_MEDIA 10     // Histórico para cálculo da média

// ----- BUFFER DO DISPLAY -----
uint8_t ssd1306_buffer[SCREEN_BUF_SIZE];

// -------------------------------------------------------------------
// ----- FONTE 5x7 (Incluindo X, Y, Z, - e .) -----
// -------------------------------------------------------------------
const uint8_t font5x7[][5] = {
    {0x7E,0x11,0x11,0x11,0x7E}, {0x00,0x21,0x7F,0x01,0x00}, {0x21,0x43,0x45,0x49,0x31}, 
    {0x42,0x41,0x51,0x69,0x46}, {0x0C,0x14,0x24,0x7F,0x04}, {0x72,0x51,0x51,0x51,0x4E}, 
    {0x1E,0x29,0x49,0x49,0x06}, {0x40,0x47,0x48,0x50,0x60}, {0x36,0x49,0x49,0x49,0x36}, 
    {0x30,0x49,0x49,0x4A,0x3C},
    {0x41,0x2A,0x1C,0x2A,0x41}, // X (Índice 10)
    {0x41,0x22,0x14,0x22,0x41}, // Y (Índice 11)
    {0x21,0x41,0x4B,0x51,0x61}, // Z (Índice 12)
    {0x00,0x00,0x1C,0x00,0x00}, // - (Índice 13)
    {0x00,0x00,0x01,0x00,0x00}  // . (Índice 14)
};


// -------------------------------------------------------------------
// ----- FUNÇÕES I2C, MPU6050, SSD1306 -----
// -------------------------------------------------------------------

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void mpu6050_init() {
    uint8_t data[2] = {0x6B, 0x00}; // PWR_MGMT_1 = 0
    i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, data, 2, 1000/portTICK_PERIOD_MS);
}

void mpu6050_read(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t data[6];
    i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, (uint8_t[]){0x3B}, 1, data, 6, 1000/portTICK_PERIOD_MS);
    *ax = (data[0]<<8)|data[1];
    *ay = (data[2]<<8)|data[3];
    *az = (data[4]<<8)|data[5];
}

void ssd1306_command(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    i2c_master_write_to_device(I2C_MASTER_NUM, SSD1306_ADDR, buf, 2, 1000/portTICK_PERIOD_MS);
}

void ssd1306_init() {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ssd1306_command(0xAE); ssd1306_command(0x20); ssd1306_command(0x00);
    ssd1306_command(0x40); ssd1306_command(0xA1); ssd1306_command(0xC8);
    ssd1306_command(0x81); ssd1306_command(0x7F); ssd1306_command(0xA6);
    ssd1306_command(0xA8); ssd1306_command(0x3F); ssd1306_command(0xD3);
    ssd1306_command(0x00); ssd1306_command(0xD5); ssd1306_command(0x80);
    ssd1306_command(0xD9); ssd1306_command(0xF1); ssd1306_command(0xDA);
    ssd1306_command(0x12); ssd1306_command(0xDB); ssd1306_command(0x40);
    ssd1306_command(0x8D); ssd1306_command(0x14); ssd1306_command(0xAF);
}

void ssd1306_clear_buffer() {
    memset(ssd1306_buffer, 0, SCREEN_BUF_SIZE);
}

void ssd1306_update() {
    for (uint8_t page = 0; page < 8; page++) {
        ssd1306_command(0xB0 + page); ssd1306_command(0x00); ssd1306_command(0x10);
        uint8_t buf[129];
        buf[0] = 0x40;
        memcpy(&buf[1], &ssd1306_buffer[page*128], 128);
        i2c_master_write_to_device(I2C_MASTER_NUM, SSD1306_ADDR, buf, 129, 1000/portTICK_PERIOD_MS);
    }
}

// -------------------------------------------------------------------
// ----- FUNÇÕES DE DESENHO PARA SSD1306 -----
// -------------------------------------------------------------------

void ssd1306_draw_char_v2(uint8_t x, uint8_t y, char c) {
    const uint8_t *ch;
    int font_index = -1;

    // Mapeamento de caracteres suportados pela sua fonte customizada
    if (c >= '0' && c <= '9') {
        font_index = c - '0';
    } else if (c == 'X') {
        font_index = 10;
    } else if (c == 'Y') {
        font_index = 11;
    } else if (c == 'Z') {
        font_index = 12;
    } else if (c == '-') {
        font_index = 13;
    } else if (c == '.') {
        font_index = 14;
    } else {
        return; 
    }
    
    ch = font5x7[font_index];

    for (uint8_t i = 0; i < 5; i++) {
        uint8_t line = ch[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (line & (1 << j)) {
                uint16_t page_offset = (y + j) / 8;
                uint16_t bit_offset = (y + j) % 8;
                uint16_t index = page_offset * 128 + x + i;
                if (index < SCREEN_BUF_SIZE) {
                    ssd1306_buffer[index] |= (1 << bit_offset);
                }
            }
        }
    }
}

void ssd1306_draw_string(uint8_t x, uint8_t y, const char *str) {
    for (int i = 0; str[i]; i++) {
        ssd1306_draw_char_v2(x + i * 6, y, str[i]);
    }
}

// Layout otimizado para exibir os 3 eixos e o estado
void ssd1306_draw_all_axes_ms2(float ax, float ay, float az, bool led_state) {
    ssd1306_clear_buffer();

    char buf[10];
    
    // --- Linha 0: Título ---
    ssd1306_draw_string(0, 0, "Aceleracao (m/s2)");
    
    // --- Linha 1: Eixo X ---
    ssd1306_draw_string(0, 16, "X:");
    snprintf(buf, sizeof(buf), "%+.2f", ax);
    // Remove o '+' se for positivo (caractere não suportado)
    if (buf[0] == '+') {
        ssd1306_draw_string(12, 16, &buf[1]);
    } else {
        ssd1306_draw_string(12, 16, buf); // Desenha com o '-'
    }

    // --- Linha 2: Eixo Y ---
    ssd1306_draw_string(0, 32, "Y:");
    snprintf(buf, sizeof(buf), "%+.2f", ay);
    if (buf[0] == '+') {
        ssd1306_draw_string(12, 32, &buf[1]);
    } else {
        ssd1306_draw_string(12, 32, buf);
    }
    
    // --- Linha 3: Eixo Z ---
    ssd1306_draw_string(0, 48, "Z:");
    snprintf(buf, sizeof(buf), "%+.2f", az);
    if (buf[0] == '+') {
        ssd1306_draw_string(12, 48, &buf[1]);
    } else {
        ssd1306_draw_string(12, 48, buf);
    }
    
    // --- Estado do LED/Movimento (Posicionado no lado direito) ---
    ssd1306_draw_string(80, 32, led_state ? "Movimento!" : "Estavel.");

    ssd1306_update();
}

// -------------------------------------------------------------------
// ----- MAIN TASK (app_main)  -----
// -------------------------------------------------------------------

void app_main(void) {
    // 1. Configuração do LED
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    // 2. Inicialização dos Periféricos
    i2c_master_init();
    mpu6050_init();
    ssd1306_init();

    // 3. Variáveis de Histórico (Para média móvel nos 3 eixos)
    int16_t ax_history[NUM_MEDIDAS_MEDIA]={0};
    int16_t ay_history[NUM_MEDIDAS_MEDIA]={0};
    int16_t az_history[NUM_MEDIDAS_MEDIA]={0};
    int idx=0;

    // Valores anteriores da média para comparação de delta nos 3 eixos
    float avg_ax_ms2_prev = 0.0;
    float avg_ay_ms2_prev = 0.0;
    float avg_az_ms2_prev = 0.0;
    
    while(1) {
        int16_t ax_raw, ay_raw, az_raw;
        
        // 4. Leitura do MPU6050
        mpu6050_read(&ax_raw, &ay_raw, &az_raw);

        // Armazena no histórico
        ax_history[idx] = ax_raw;
        ay_history[idx] = ay_raw; 
        az_history[idx] = az_raw; 
        idx = (idx+1)%NUM_MEDIDAS_MEDIA;

        // 5. Cálculo da Média (RAW) para os 3 eixos
        int32_t sum_ax=0, sum_ay=0, sum_az=0;
        for(int i=0; i<NUM_MEDIDAS_MEDIA; i++){
            sum_ax+=ax_history[i];
            sum_ay+=ay_history[i];
            sum_az+=az_history[i];
        }
        float avg_ax_raw = (float)sum_ax / NUM_MEDIDAS_MEDIA;
        float avg_ay_raw = (float)sum_ay / NUM_MEDIDAS_MEDIA;
        float avg_az_raw = (float)sum_az / NUM_MEDIDAS_MEDIA;

        // 6. Conversão para m/s²
        float avg_ax_ms2 = (avg_ax_raw / MPU_SCALE_FACTOR) * G_TO_MS2;
        float avg_ay_ms2 = (avg_ay_raw / MPU_SCALE_FACTOR) * G_TO_MS2;
        float avg_az_ms2 = (avg_az_raw / MPU_SCALE_FACTOR) * G_TO_MS2;

        // 7. Saída Serial
        printf("Media MS2: X:%+.2f\tY:%+.2f\tZ:%+.2f\n", avg_ax_ms2, avg_ay_ms2, avg_az_ms2);

        // 8. Lógica do LED: Delta de 0.5 m/s² em QUALQUER EIXO
        bool movimento_detectado = false;
        
        if(fabs(avg_ax_ms2 - avg_ax_ms2_prev) >= DELTA_MS2_LIMITE || 
           fabs(avg_ay_ms2 - avg_ay_ms2_prev) >= DELTA_MS2_LIMITE || 
           fabs(avg_az_ms2 - avg_az_ms2_prev) >= DELTA_MS2_LIMITE) 
        {
            movimento_detectado = true;
        }

        if(movimento_detectado){
            gpio_set_level(LED_GPIO, 1);
            // Atualiza TODOS os valores anteriores
            avg_ax_ms2_prev = avg_ax_ms2; 
            avg_ay_ms2_prev = avg_ay_ms2; 
            avg_az_ms2_prev = avg_az_ms2; 
        } else {
            gpio_set_level(LED_GPIO, 0);
        }
        
        // 9. Atualiza Display
        ssd1306_draw_all_axes_ms2(avg_ax_ms2, avg_ay_ms2, avg_az_ms2, movimento_detectado);

        // 10. Atraso (200 ms)
        vTaskDelay(pdMS_TO_TICKS(INTERVALO_MS));
    }
}