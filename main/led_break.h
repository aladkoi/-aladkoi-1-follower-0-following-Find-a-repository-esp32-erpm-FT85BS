#include <stdio.h>
#include <driver/gpio.h>
#include <driver/ledc.h>

#define BREAK_LIGHT_PIN GPIO_NUM_5 // Замените на ваш пин

void initBreakLight() {
    // Настройка пина для BREAK_LIGHT_PIN
    // Настройка LEDC
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 5000,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0
    };
    ledc_timer_config(&ledc_timer);

    // Привязка пина к LEDC
    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL_0,
        .gpio_num = BREAK_LIGHT_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0 // Измените на 128, чтобы установить яркость 50%
    };
    ledc_channel_config(&ledc_channel);
}

void DoLight(int data) {
       ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, data); 
       ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
}

