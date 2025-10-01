#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"

#define LED1_GPIO 10
#define LED2_GPIO 11
#define LED3_GPIO 12
#define LED4_GPIO 13
#define BUZZER_GPIO 18

#define DELAY_MS 20 // Tempo entre atualizações

#define LED_PWM_FREQ     1000 // 1 kHz para LEDs
#define BUZZER_FREQ_INIT 1000 // Frequência inicial do buzzer
#define BUZZER_FREQ_MIN  500
#define BUZZER_FREQ_MAX  2000

#define PWM_RESOLUTION LEDC_TIMER_10_BIT // 0-255

void init_pwm_channels() {
    // Timer para LEDs
    ledc_timer_config_t led_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = LED_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&led_timer);

    // Timer para buzzer
    ledc_timer_config_t buzzer_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = BUZZER_FREQ_INIT,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&buzzer_timer);

    // Canais LED (usam o mesmo timer)
    int led_gpios[] = {LED1_GPIO, LED2_GPIO, LED3_GPIO, LED4_GPIO};
    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t led_channel = {
            .channel    = i, // LEDC_CHANNEL_0 até 3
            .duty       = 0,
            .gpio_num   = led_gpios[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_0
        };
        ledc_channel_config(&led_channel);
    }

    // Canal do buzzer (canal 4)
    ledc_channel_config_t buzzer_channel = {
        .channel    = LEDC_CHANNEL_4,
        .duty       = 0, // 50% duty
        .gpio_num   = BUZZER_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_1
    };
    ledc_channel_config(&buzzer_channel);
}

void set_led_duty(int channel, int duty_percent) {
    uint32_t duty = (255 * duty_percent) / 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

void set_buzzer_freq(int freq_hz) {
    uint32_t real_freq = ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_1, freq_hz);
    printf("Freq pedida: %d Hz\n", freq_hz);

    int duty = (1 << PWM_RESOLUTION) / 2; // 50% duty
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);
}
void fading_sync_phase() {
    for (int duty = 0; duty <= 100; duty += 5) {
        for (int ch = 0; ch < 4; ch++) {
            set_led_duty(ch, duty);
        }
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }
    for (int duty = 100; duty >= 0; duty -= 5) {
        for (int ch = 0; ch < 4; ch++) {
            set_led_duty(ch, duty);
        }
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }
}

void fading_sequential_phase() {
    for (int ch = 0; ch < 4; ch++) {
        for (int duty = 0; duty <= 100; duty += 5) {
            set_led_duty(ch, duty);
            vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
        }
        for (int duty = 100; duty >= 0; duty -= 5) {
            set_led_duty(ch, duty);
            vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
        }
    }

    // Retorno
    for (int ch = 3; ch >= 0; ch--) {
        for (int duty = 0; duty <= 100; duty += 5) {
            set_led_duty(ch, duty);
            vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
        }
        for (int duty = 100; duty >= 0; duty -= 5) {
            set_led_duty(ch, duty);
            vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
        }
    }
}
void stop_buzzer() {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);
}

void buzzer_test_phase() {
    for (int freq = BUZZER_FREQ_MIN; freq <= BUZZER_FREQ_MAX; freq += 50) {
        set_buzzer_freq(freq);
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }
    for (int freq = BUZZER_FREQ_MAX; freq >= BUZZER_FREQ_MIN; freq -= 50) {
        set_buzzer_freq(freq);
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }
    stop_buzzer();
}


void app_main() {
  init_pwm_channels();
  while (1) {
    fading_sync_phase();
    fading_sequential_phase();
    buzzer_test_phase();
  }
}
