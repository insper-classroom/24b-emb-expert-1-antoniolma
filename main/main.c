/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

const int PWM_0_PIN = 14;
const int PWM_1_PIN = 15;

QueueHandle_t xQueueServoX;
QueueHandle_t xQueueServoY;

void adc_1_task(void *p) {
    adc_init();
    adc_gpio_init(27);
    adc_gpio_init(26);

    // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
    // const float conversion_factor = 3.3f / (1 << 12);   

    // uint16_t result;
    uint16_t last_x_pulse = 0;
    uint16_t last_y_pulse = 0;
    while (1) {
        adc_select_input(1); // Select ADC input 1 (GPIO27)
        uint16_t adc_read_x = adc_read();
        uint16_t servo_x_pulse = 400 + (adc_read_x * 2000) / 4095;

        // printf("read x: %f\n", (float) adc_read_x);
        // printf("last x: %f\n", (float) last_x);
        if ((servo_x_pulse > last_x_pulse && (servo_x_pulse - last_x_pulse) >= 5) ||
            (servo_x_pulse < last_x_pulse && (last_x_pulse - servo_x_pulse) >= 5)) {
            // printf("servo x: %f\n", (float) servo_x_pulse);
            xQueueSend(xQueueServoX, &servo_x_pulse, 0);
            last_x_pulse = servo_x_pulse;
            // printf("new X\n");
        }

        adc_select_input(0); // Select ADC input 0 (GPIO26)
        uint16_t adc_read_y = adc_read();
        uint16_t servo_y_pulse = 400 + (adc_read_y * 2000) / 4095;
        // printf("read y: %f\n", (float) adc_read_y);
        // printf("last y: %f\n", (float) last_y);
        if ((servo_y_pulse > last_y_pulse && (servo_y_pulse - last_y_pulse) >= 5) ||
            (servo_y_pulse < last_y_pulse && (last_y_pulse - servo_y_pulse) >= 5)) {
            // printf("servo y: %f\n", (float) servo_y_pulse);
            xQueueSend(xQueueServoY, &servo_y_pulse, 0);
            last_y_pulse = servo_y_pulse;
            // printf("new Y\n");
        }

        // CÓDIGO AQUI

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ==================================================================================

void setMillis(int servoPin, float millis)
{
    float wrap = 39062;
    pwm_set_gpio_level(servoPin, (millis/20000.f)*wrap);
}

void setServo(int servoPin, float startMillis)
{
    gpio_set_function(servoPin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(servoPin);

    pwm_config config = pwm_get_default_config();
    
    uint64_t clockspeed = 125000000;
    float clockDiv = 64;
    float wrap = 39062;

    while (clockspeed/clockDiv/50 > 65535 && clockDiv < 256) clockDiv += 64; 
    wrap = clockspeed/clockDiv/50;

    pwm_config_set_clkdiv(&config, clockDiv);
    pwm_config_set_wrap(&config, wrap);

    pwm_init(slice_num, &config, true);

    setMillis(servoPin, startMillis);
}

void servo_x_task(void *p) {
    // bool directionOne = true;
    float currentMillisOne = 0;

    setServo(PWM_1_PIN, currentMillisOne);

    uint16_t pulse;
    while (true) {
        // currentMillisOne += (directionOne)?5:-5;
        // if (currentMillisOne >= 2400) directionOne = false;   // Se tiver indo pra mais que 180 graus, vira
        // if (currentMillisOne <= 400) directionOne = true;     // Se tiver indo pra menos que 0 graus, vira

        if (xQueueReceive(xQueueServoX, &pulse, pdMS_TO_TICKS(0))) {
            currentMillisOne = (float) pulse;
            printf("Pulse x: %f\n", (float) pulse);
            printf("currentMillis X: %f\n", currentMillisOne);
        }

        setMillis(PWM_1_PIN, currentMillisOne);
    }
}

void servo_y_task(void *p) {
    // bool directionOne = true;
    float currentMillisOne = 0;

    setServo(PWM_0_PIN, currentMillisOne);

    uint16_t pulse;
    while (true) {
        // currentMillisOne += (directionOne)?5:-5;
        // if (currentMillisOne >= 2400) directionOne = false;   // Se tiver indo pra mais que 180 graus, vira
        // if (currentMillisOne <= 400) directionOne = true;     // Se tiver indo pra menos que 0 graus, vira

        if (xQueueReceive(xQueueServoY, &pulse, pdMS_TO_TICKS(0))) {
            currentMillisOne = (float) pulse;
            printf("Pulse x: %f\n", (float) pulse);
            printf("currentMillis X: %f\n", currentMillisOne);
        }

        setMillis(PWM_0_PIN, currentMillisOne);
    }
}

int main() {
    stdio_init_all();
    printf("Start RTOS \n");
    adc_init();

    xQueueServoX = xQueueCreate(32, sizeof(uint16_t));
    xQueueServoY = xQueueCreate(32, sizeof(uint16_t));

    xTaskCreate(adc_1_task, "ADC_TASK 1", 4095, NULL, 1, NULL);
    xTaskCreate(servo_x_task, "SERVO_TASK 1", 256, NULL, 1, NULL);
    xTaskCreate(servo_y_task, "SERVO_TASK 2", 256, NULL, 1, NULL);
    vTaskStartScheduler();

    while (true) {
    }
}
