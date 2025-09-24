// LINK : https://wokwi.com/projects/442978990925868033

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED1_GPIO 10
#define LED2_GPIO 11
#define LED3_GPIO 12
#define LED4_GPIO 13

#define DELAY_MS 500

// Inicializa os GPIOs dos LEDs
void init_leds() {
    gpio_reset_pin(LED1_GPIO);
    gpio_set_direction(LED1_GPIO, GPIO_MODE_OUTPUT);

    gpio_reset_pin(LED2_GPIO);
    gpio_set_direction(LED2_GPIO, GPIO_MODE_OUTPUT);

    gpio_reset_pin(LED3_GPIO);
    gpio_set_direction(LED3_GPIO, GPIO_MODE_OUTPUT);

    gpio_reset_pin(LED4_GPIO);
    gpio_set_direction(LED4_GPIO, GPIO_MODE_OUTPUT);

    // Apaga todos os LEDs no início
    gpio_set_level(LED1_GPIO, 0);
    gpio_set_level(LED2_GPIO, 0);
    gpio_set_level(LED3_GPIO, 0);
    gpio_set_level(LED4_GPIO, 0);
}

// Controla um LED específico
void set_led(int led_num, int state) {
    switch (led_num) {
        case 1: gpio_set_level(LED1_GPIO, state); break;
        case 2: gpio_set_level(LED2_GPIO, state); break;
        case 3: gpio_set_level(LED3_GPIO, state); break;
        case 4: gpio_set_level(LED4_GPIO, state); break;
    }
}

// Exibe um número binário nos LEDs
void binary_counter_phase() {
    for (int i = 0; i < 16; i++) {
        gpio_set_level(LED1_GPIO, (i >> 0) & 1);
        gpio_set_level(LED2_GPIO, (i >> 1) & 1);
        gpio_set_level(LED3_GPIO, (i >> 2) & 1);
        gpio_set_level(LED4_GPIO, (i >> 3) & 1);

        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }
}

// Sequência de varredura LED1 -> LED4 e volta
void scanning_phase() {
    for (int i = 1; i <= 4; i++) {
        set_led(i, 1);
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
        set_led(i, 0);
    }
    for (int i = 4; i >= 1; i--) {
        set_led(i, 1);
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
        set_led(i, 0);
    }
}

void app_main(void) {
    init_leds();

    while (1) {
        binary_counter_phase();
        scanning_phase();
    }
}
