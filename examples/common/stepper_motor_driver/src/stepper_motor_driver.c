/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee temperature sensor driver example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#include "stepper_motor_driver.h"

#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "esp_attr.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "driver/gptimer.h"
#include <driver/gpio.h>

static const char *TAG = "ESP_STEPPER_DRIVER";

#define TOTAL_STEPS 4096 // or whatever is appropriate for your stepper
#define TOTAL_PATTERNS 8
#define NUM_PINS 4

static QueueHandle_t stepper_queue;
typedef struct {
    uint64_t event_count;
    stepper_driver *driver;
} stepper_event_t;
gptimer_handle_t gptimer;

// Precomputed step patterns
static const int16_t step_patterns[TOTAL_PATTERNS][NUM_PINS] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1}
};

static bool timer_on_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_awoken = pdFALSE;
    stepper_event_t event = {
        .event_count = edata->count_value,
        .driver = (stepper_driver *)user_ctx  // Retrieve driver reference
    };
    // Send event to the queue
    xQueueSendFromISR(stepper_queue, &event, &high_task_awoken);
    // Return whether we need to yield at the end of ISR
    return high_task_awoken == pdTRUE;
}

static void stepper_task(void *arg)
{
    stepper_event_t event;

    while (true) {
        // Wait for events from the ISR
        if (xQueueReceive(stepper_queue, &event, portMAX_DELAY)) {
            // Example operation on the driver
            step(event.driver);  // Replace with actual function
        }
    }
}

void init_timer(stepper_driver *driver)
{
    stepper_queue = xQueueCreate(10, sizeof(stepper_event_t));
    assert(stepper_queue != NULL);

    // Create the stepper task
    xTaskCreate(stepper_task, "Stepper Task", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "Create timer handle");
    gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_alarm_config_t alarm_config1 = {
        .reload_count = 0,
        .alarm_count = driver->delay_us, // period = 1s
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config1));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_on_alarm_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, driver));

    ESP_LOGI(TAG, "Enable timer");
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_LOGI(TAG, "Start timer");
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}

void deinit_timer()
{
    ESP_LOGI(TAG, "Stop timer");
    ESP_ERROR_CHECK(gptimer_stop(gptimer));
    ESP_LOGI(TAG, "Disable timer");
    ESP_ERROR_CHECK(gptimer_disable(gptimer));
    ESP_LOGI(TAG, "Delete timer");
    ESP_ERROR_CHECK(gptimer_del_timer(gptimer));
    gptimer = NULL;
}

void init_stepper_driver(stepper_driver *driver, int16_t in1, int16_t in2, int16_t in3, int16_t in4) {
    driver->pins[0] = in1;
    driver->pins[1] = in2;
    driver->pins[2] = in3;
    driver->pins[3] = in4;
    driver->seq_n = 0;
    driver->step_n = 0;
    driver->clockwise = true;
    driver->delay_us = 2000;
    driver->task_handle = NULL;

    for (int16_t pin = 0; pin < 4; pin++) {
        gpio_set_direction(driver->pins[pin], GPIO_MODE_OUTPUT);
    }
}

void set_rpm(stepper_driver *driver, int16_t rpm) {
    if (rpm < 6) return;
    if (rpm >= 24) {
        driver->delay_us = 1000;
        return;
    }
    driver->delay_us = (60 * 1000000UL) / (TOTAL_STEPS * rpm);
}

void stop_move_task(stepper_driver *driver) {
    ESP_LOGW(TAG, "Stop Move Task\n");
    driver->running = false;
    if (driver->task_handle != NULL) {
        vTaskDelete(driver->task_handle);
        driver->task_handle = NULL;
    }
    if(gptimer != NULL) deinit_timer();
}

void start_move_task(stepper_driver *driver, bool clockwise) {
    driver->clockwise = clockwise;
    driver->running = true;
    if (driver->task_handle != NULL) stop_move_task(driver);
    if (driver->clockwise) 
        ESP_LOGW(TAG, "Start Move Up\n");
    else
        ESP_LOGW(TAG, "Start Move Down\n");
    xTaskCreate(move_task, "move_stepper_motor", 4096, driver, 10,  &(driver->task_handle));

    if(gptimer != NULL) deinit_timer();
    init_timer(driver);
}

void move_task(void *param)
{
    stepper_driver *driver = (stepper_driver *)param;

    while(driver->running) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    vTaskDelete(driver->task_handle);
}

void step(stepper_driver *driver) {
    // Update the sequence number
    if (driver->clockwise) {
        driver->seq_n++;
        if (driver->seq_n >= TOTAL_PATTERNS) driver->seq_n = 0;
    } else {
        driver->seq_n--;
        if (driver->seq_n < 0) driver->seq_n = TOTAL_PATTERNS - 1;
    }

    // Retrieve the precomputed pattern
    const int16_t *pattern = step_patterns[driver->seq_n];

    // Set GPIO levels in one loop
    for (int16_t p = 0; p < NUM_PINS; p++) {
        gpio_set_level(driver->pins[p], pattern[p]);
    }

    // Update the step count
    driver->step_n += driver->clockwise ? 1 : -1;
    if (driver->step_n >= TOTAL_STEPS) driver->step_n -= TOTAL_STEPS;
    if (driver->step_n < 0) driver->step_n += TOTAL_STEPS;
}