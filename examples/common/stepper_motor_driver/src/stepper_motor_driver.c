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

static esp_step_callback_t func_ptr;

#define TOTAL_STEPS 4096 // or whatever is appropriate for your stepper
#define TOTAL_PATTERNS 8
#define NUM_PINS 4

static QueueHandle_t stepper_queue;
typedef struct {
    uint64_t event_count;
    stepper_driver *driver;
} stepper_event_t;
gptimer_handle_t gptimer;

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

    if (xQueueReceive(stepper_queue, &event, portMAX_DELAY)) {
        event.driver->clockwise = set_direction(event.driver);
    }

    while (true) {
        if (xQueueReceive(stepper_queue, &event, portMAX_DELAY)) {
            if ((!event.driver->clockwise && event.driver->step_goal >= event.driver->step_n) ||
                (event.driver->clockwise && event.driver->step_goal <= event.driver->step_n)) {
                stop_move_task(event.driver);
            }
            step(event.driver); 
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

void init_stepper_driver(stepper_driver *driver, int16_t pin_step, int16_t pin_dir, int16_t pin_sleep, esp_step_callback_t cb) {
    driver->pin_dir = pin_dir;
    driver->pin_step = pin_step;
    driver->pin_sleep = pin_sleep;
    driver->step_n = 0;
    driver->step_min = 0;
    driver->step_max = 200 * 30;
    driver->step_goal = 0;
    driver->delay_us = 2000;
    driver->task_handle = NULL;

    func_ptr = cb;

    gpio_set_direction(driver->pin_dir, GPIO_MODE_OUTPUT);
    gpio_set_direction(driver->pin_step, GPIO_MODE_OUTPUT);
    gpio_set_direction(driver->pin_sleep, GPIO_MODE_OUTPUT);

    /* init in sleep mode */
    gpio_set_level(driver->pin_sleep, 1);
}

void set_rpm(stepper_driver *driver, int16_t rpm) {
    if (rpm < 6) return;
    if (rpm >= 24) {
        driver->delay_us = 1000;
        return;
    }
    driver->delay_us = 1000; // (60 * 1000000UL) / (TOTAL_STEPS * rpm);
}

void stop_move_task(stepper_driver *driver) {
    ESP_LOGW(TAG, "Stop Move Task\n");
    gpio_set_level(driver->pin_sleep, 1);
    driver->running = false;
    if (driver->task_handle != NULL) {
        vTaskDelete(driver->task_handle);
        driver->task_handle = NULL;
    }
    if(gptimer != NULL) deinit_timer();
}

void start_move_task(stepper_driver *driver, int8_t percentage) {
    driver->step_goal = (percentage / 100.0) * (driver->step_max - driver->step_min) ;
    driver->running = true;
    if (driver->task_handle != NULL) stop_move_task(driver);
    if (driver->step_goal > driver->step_n){
        ESP_LOGW(TAG, "Start Move Up\n");
    } else {
        ESP_LOGW(TAG, "Start Move Down\n");
    }
    gpio_set_level(driver->pin_sleep, 0);
    vTaskDelay(1);
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
    
bool set_direction(stepper_driver *driver) {
    if(driver->step_goal > driver->step_n){
        gpio_set_level(driver->pin_dir, 0);
        return true;
    } else {
        gpio_set_level(driver->pin_dir, 1);
        return false;
    }
}

void step(stepper_driver *driver) {
    if(driver->toggle_step){
        gpio_set_level(driver->pin_step, 1);
    } else {
        gpio_set_level(driver->pin_step, 0);
    }

    driver->toggle_step = !driver->toggle_step;

    // Update the step count
    driver->step_n += driver->clockwise ? 1 : -1;

    (*func_ptr)(&driver->step_n, &driver->step_min, &driver->step_max);
}