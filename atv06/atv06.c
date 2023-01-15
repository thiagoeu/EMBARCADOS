// link: https://wokwi.com/projects/444245245960873985

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"

// PINOS
#define LED1 GPIO_NUM_15
#define LED2 GPIO_NUM_16
#define LED3 GPIO_NUM_17
#define LED4 GPIO_NUM_18

#define BUTTON_A GPIO_NUM_20
#define BUTTON_B GPIO_NUM_21

// CONSTANTES
#define DEBOUNCE_TIME_US 200000  // 200 ms

// VARIÁVEIS GLOBAIS
static uint8_t contador = 0;
static uint8_t incremento = 1;

static int64_t last_press_a = 0;
static int64_t last_press_b = 0;

// FUNÇÕES
void atualizar_leds(uint8_t valor) {
    gpio_set_level(LED1, (valor >> 0) & 0x01);
    gpio_set_level(LED2, (valor >> 1) & 0x01);
    gpio_set_level(LED3, (valor >> 2) & 0x01);
    gpio_set_level(LED4, (valor >> 3) & 0x01);
}

static void IRAM_ATTR isr_botao_a(void* arg) {
    int64_t now = esp_timer_get_time();
    if (now - last_press_a > DEBOUNCE_TIME_US) {
        last_press_a = now;
        contador = (contador + incremento) & 0x0F;  // Circular entre 0x0 e 0xF
        atualizar_leds(contador);
    }
}

static void IRAM_ATTR isr_botao_b(void* arg) {
    int64_t now = esp_timer_get_time();
    if (now - last_press_b > DEBOUNCE_TIME_US) {
        last_press_b = now;
        incremento = (incremento == 1) ? 2 : 1;
    }
}

void configurar_gpio() {
    // SAÍDA DOS LEDS
    gpio_reset_pin(LED1);
    gpio_reset_pin(LED2);
    gpio_reset_pin(LED3);
    gpio_reset_pin(LED4);

    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED3, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED4, GPIO_MODE_OUTPUT);

    // ENTRADAS DOS BOTÕES
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_A) | (1ULL << BUTTON_B),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };

    gpio_config(&io_conf);  // <- corrigido aqui

    // INTERRUPÇÕES
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_A, isr_botao_a, NULL); // <- corrigido aqui
    gpio_isr_handler_add(BUTTON_B, isr_botao_b, NULL); // <- corrigido aqui

    atualizar_leds(contador);
}

void app_main() {
    configurar_gpio();
    atualizar_leds(contador);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
