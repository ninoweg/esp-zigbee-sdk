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

#include "nvs_flash.h"
#include "nvs.h"

#define NAMESPACE_STORAGE "cover_storage"
#define CURRENT_STEP_KEY "cur_step"
#define MINIMUM_STEP_KEY "min_step"
#define MAXIMUM_STEP_KEY "max_step"

static const char *TAG = "ESP_STEPPER_DRIVER";

static esp_step_callback_t func_ptr;

#define TOTAL_STEPS 200

static QueueHandle_t stepper_queue;
typedef struct {
    uint64_t event_count;
    stepper_driver *driver;
} stepper_event_t;
gptimer_handle_t gptimer;

typedef enum {
    TIMER_STATE_RUNNING,
    TIMER_STATE_DEINITIALIZED,
    TIMER_STATE_INITIALIZED,
} timer_state_t;

static timer_state_t timer_state = TIMER_STATE_DEINITIALIZED;

static bool timer_on_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    BaseType_t high_task_awoken = pdFALSE;
    stepper_event_t event = {
        .event_count = edata->count_value,
        .driver = (stepper_driver *)user_ctx
    };
    xQueueSendFromISR(stepper_queue, &event, &high_task_awoken);
    return high_task_awoken == pdTRUE;
}

static void move_task(void *param) {

    stepper_driver *driver = (stepper_driver *)param;

    stepper_event_t event;

    for (;;) {
        if (driver->stop_task) {
            stop_move_task(driver);
            driver->task_handle = NULL;
            vTaskDelete(NULL);
        }

        if (xQueueReceive(stepper_queue, &event, portMAX_DELAY)) {
            // Check if goal was reached
            if ((!driver->pos_dir && driver->step_n <= driver->step_goal) ||
                (driver->pos_dir && driver->step_n >= driver->step_goal)) {
                driver->stop_task = true;
            } else {
                // Make next step
                step(driver);
                // Call cb function to update attributes
                if (!(driver->step_n % 100) ||
                    (!driver->pos_dir && driver->step_goal == driver->step_n + 1) ||
                    (driver->pos_dir && driver->step_goal == driver->step_n - 1)) {
                    (*func_ptr)(&driver->step_n, &driver->step_min, &driver->step_max);
                }
            }
            
        }
    }
}

static void save_driver_params(stepper_driver *driver)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(NAMESPACE_STORAGE, NVS_READWRITE, &my_handle);

    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return;
    }

    // Write
    err = nvs_set_u16(my_handle, CURRENT_STEP_KEY, driver->step_n);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Could not save current step");
    } else {
        ESP_LOGI(TAG, "Saved current step at %d.", driver->step_n);
    }

    err = nvs_set_u16(my_handle, MINIMUM_STEP_KEY, driver->step_min);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Could not save minimum step");
    } else {
        ESP_LOGI(TAG, "Saved minimum step at %d.", driver->step_min);
    }

    err = nvs_set_u16(my_handle, MAXIMUM_STEP_KEY, driver->step_max);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Could not save maximum step");
    } else {
        ESP_LOGI(TAG, "Saved maximum step at %d.", driver->step_max);
    }


    // Commit written value
    ESP_LOGI(TAG, "Committing updates in NVS ... ");
    err = nvs_commit(my_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Failed!");
    } else {
        ESP_LOGI(TAG, "Done");
    }
    // Close
    nvs_close(my_handle);
}

static void load_driver_params(stepper_driver *driver)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(NAMESPACE_STORAGE, NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return;
    }

    // Read
    err = nvs_get_u16(my_handle, CURRENT_STEP_KEY, &driver->step_n);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Could not update current step");
    } else {
        ESP_LOGI(TAG, "Updated current step to %d.", driver->step_n);
    }

    err = nvs_get_u16(my_handle, MINIMUM_STEP_KEY, &driver->step_min);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Could not update minimum step");
    } else {
        ESP_LOGI(TAG, "Updated minimum step to %d.", driver->step_min);
    }

    err = nvs_get_u16(my_handle, MAXIMUM_STEP_KEY, &driver->step_max);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Could not update maximum step");
    } else {
        ESP_LOGI(TAG, "Updated maximum step to %d.", driver->step_max);
    }

    // Close
    nvs_close(my_handle);
}

void init_timer(stepper_driver *driver) {
    stepper_queue = xQueueCreate(10, sizeof(stepper_event_t));
    assert(stepper_queue != NULL);

    // Create timer
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

    timer_state = TIMER_STATE_INITIALIZED; 
}

void start_timer() {
    if (timer_state != TIMER_STATE_INITIALIZED || gptimer == NULL) return;

    ESP_LOGI(TAG, "Enable timer");
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_LOGI(TAG, "Start timer");
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    timer_state = TIMER_STATE_RUNNING;
}

void stop_timer() {
    if (timer_state != TIMER_STATE_RUNNING || gptimer == NULL) return;

    ESP_LOGI(TAG, "Stop timer");
    ESP_ERROR_CHECK(gptimer_stop(gptimer));
    ESP_LOGI(TAG, "Disable timer");
    ESP_ERROR_CHECK(gptimer_disable(gptimer));

    timer_state = TIMER_STATE_INITIALIZED;
}

void delete_timer() {
    if (timer_state == TIMER_STATE_DEINITIALIZED || gptimer == NULL) return;
    if (timer_state == TIMER_STATE_RUNNING) stop_timer();
    ESP_LOGI(TAG, "Delete timer");
    ESP_ERROR_CHECK(gptimer_del_timer(gptimer));
    
    gptimer = NULL;
    timer_state = TIMER_STATE_DEINITIALIZED;
}

void init_stepper_driver(stepper_driver *driver, uint8_t pin_step, uint8_t pin_dir, uint8_t pin_sleep, esp_step_callback_t cb) {
    driver->pin_dir = pin_dir;
    driver->pin_step = pin_step;
    driver->pin_sleep = pin_sleep;
    driver->step_n = 1;
    driver->step_min = 0;
    driver->step_max = 6000;
    driver->step_goal = 0;
    driver->delay_us = 2000;
    driver->task_handle = NULL;
    driver->pos_dir = true;

    load_driver_params(driver);

    func_ptr = cb;

    gpio_set_direction(driver->pin_dir, GPIO_MODE_OUTPUT);
    gpio_set_direction(driver->pin_step, GPIO_MODE_OUTPUT);
    gpio_set_direction(driver->pin_sleep, GPIO_MODE_OUTPUT);

    /* init in sleep mode */
    gpio_set_level(driver->pin_sleep, 1);

    // Init timer
    init_timer(driver);
}

void set_rpm(stepper_driver *driver, uint16_t rpm) {
    if (rpm < 30) {
        driver->delay_us = 10000;
        return;
    } 
    if (rpm >= 300) {
        driver->delay_us = 1000;
        return;
    }
    driver->delay_us = (60 * 1000000UL) / (TOTAL_STEPS * rpm);
}

void stop_move_task(stepper_driver *driver) {
    ESP_LOGW(TAG, "Stop Move Task\n");
    gpio_set_level(driver->pin_sleep, 1);
    if (driver->task_handle != NULL) {
        driver->stop_task = true;
    }
    if (gptimer != NULL) stop_timer();

    save_driver_params(driver);
    (*func_ptr)(&driver->step_n, &driver->step_min, &driver->step_max);
}

void start_move_task(stepper_driver *driver, uint8_t percentage) {
    // Calculate step goal from percentage
    driver->step_goal = (percentage / 100.0) * (driver->step_max - driver->step_min);
    driver->step_goal = driver->step_goal > driver->step_max ? driver->step_max : driver->step_goal < driver->step_min ? driver->step_min : driver->step_goal; 
    
    // Check for running move task
    if (driver->task_handle != NULL) driver->stop_task = true;

    // Wait for task to finish
    for(size_t i = 0; i < 10; ++i) {
        if(driver->task_handle != NULL) vTaskDelay(10);
        else break;
    }
    // return if task did not finish
    if (driver->task_handle != NULL) return;
    
    // Enable stepper driver
    gpio_set_level(driver->pin_sleep, 0);

    // Reset the stop_task flag
    driver->stop_task = false;

    // Calculate required turning direction
    driver->pos_dir = set_direction(driver);

    // Enable timer
    start_timer();

    // Create the stepper task
    ESP_LOGW(TAG, "Start Move Task\n");
    xTaskCreate(move_task, "Move Task", 4096, driver, 10, &(driver->task_handle));
}
    
bool set_direction(stepper_driver *driver) {
    bool dir = driver->step_goal > driver->step_n;
    gpio_set_level(driver->pin_dir, dir);
    return dir;
}

void step(stepper_driver *driver) {
    static int8_t interval = 0;
    bool level = driver->toggle_step ? 1 : 0;
    gpio_set_level(driver->pin_step, level);
    driver->toggle_step = !driver->toggle_step;

    if (!(interval % 10))
        driver->step_n += driver->pos_dir ? 1 : -1;
    interval++;
}