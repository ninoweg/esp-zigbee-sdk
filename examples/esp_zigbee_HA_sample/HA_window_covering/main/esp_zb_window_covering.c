/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee HA_window_covering Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#include "esp_zb_window_covering.h"
#include "stepper_motor_driver.h" // custom stepper motor driver

#include "stdio.h"
#include "string.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "iot_button.h"
#include "esp_idf_version.h"

#define BOOT_BUTTON_NUM         9
#define BUTTON_ACTIVE_LEVEL     0

#define ARRAY_LENTH(arr) (sizeof(arr) / sizeof(arr[0]))

#define CONFIG_GPIO_BUTTON_SUPPORT_POWER_SAVE 1

#if defined ZB_ED_ROLE
#error Define ZB_COORDINATOR_ROLE in idf.py menuconfig to compile window covering source code.
#endif

static const char *TAG = "ESP_ZB_WINDOW_COVERING";
static stepper_driver stepper_motor_driver;

static const char *TAG_BUTTON = "BUTTON_POWER_SAVE";

const char *button_event_table[] = {
    "BUTTON_PRESS_DOWN",
    "BUTTON_PRESS_UP",
    "BUTTON_PRESS_REPEAT",
    "BUTTON_PRESS_REPEAT_DONE",
    "BUTTON_SINGLE_CLICK",
    "BUTTON_DOUBLE_CLICK",
    "BUTTON_MULTIPLE_CLICK",
    "BUTTON_LONG_PRESS_START",
    "BUTTON_LONG_PRESS_HOLD",
    "BUTTON_LONG_PRESS_UP",
};

char modelid[] = {24, 'E', 'S', 'P', '3', '2', '-', 'C', '6', '-', 'W', 'i', 'n', 'd', 'o', 'w', '-', 'C', 'o', 'v', 'e', 'r', 'i', 'n', 'g'};
char manufname[] = {9, 'E', 's', 'p', 'r', 'e', 's', 's', 'i', 'f'};

#define BOOT_BUTTON_NUM 9

/********************* Define functions **************************/

static void button_event_cb(void *arg, void *data)
{
    ESP_LOGI(TAG_BUTTON, "Button event %s", button_event_table[(button_event_t)data]);
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_UNDEFINED) {
        ESP_LOGI(TAG_BUTTON, "Wake up from light sleep, reason %d", cause);
    }
}

void button_init(uint32_t button_num)
{
    button_config_t btn_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config = {
            .gpio_num = button_num,
            .active_level = BUTTON_ACTIVE_LEVEL
#if CONFIG_GPIO_BUTTON_SUPPORT_POWER_SAVE
            .enable_power_save = true,
#endif
        },
    };
    button_handle_t btn = iot_button_create(&btn_cfg);
    
    assert(btn);
    esp_err_t err = iot_button_register_cb(btn, BUTTON_SINGLE_CLICK, button_event_cb, (void *)BUTTON_SINGLE_CLICK);
    err |= iot_button_register_cb(btn, BUTTON_DOUBLE_CLICK, button_event_cb, (void *)BUTTON_DOUBLE_CLICK);
    ESP_ERROR_CHECK(err);
}

static void update_step_handler(int16_t* step, int16_t* min, int16_t* max)
{
    esp_zb_lock_acquire(portMAX_DELAY);
    static int8_t last_set_lift_percentage = 0;
    int16_t lift_value = *step;
    int16_t lift_total_steps = *max - *min;
    if (lift_total_steps == 0) {
        ESP_LOGW(TAG, "Limits recalibration required.");
        return;
    }
    
    int8_t lift_percentage = (100.0 * lift_value) / lift_total_steps;

    if (abs(last_set_lift_percentage - lift_percentage) >= 1) {
        ESP_LOGI(TAG, "Step Number: %d", lift_value);
        ESP_LOGI(TAG, "Percentage: %d", lift_percentage);

        esp_zb_zcl_set_attribute_val(HA_WINDOW_COVERING_ENDPOINT,
                                    ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, 
                                    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                    ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID, 
                                    &lift_percentage, 
                                    false);
        esp_zb_zcl_set_attribute_val(HA_WINDOW_COVERING_ENDPOINT,
                                    ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, 
                                    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                    ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_ID, 
                                    &lift_value, 
                                    false);
        last_set_lift_percentage = lift_percentage;
    }
    esp_zb_lock_release();
}

static esp_err_t deferred_driver_init(void)
{
    /* Initialize stepper driver */
    init_stepper_driver(&stepper_motor_driver, 1, 2, 3, &update_step_handler);
    set_rpm(&stepper_motor_driver, 12);
    button_init(BOOT_BUTTON_NUM);
    return ESP_OK;
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
            }
        } else {
            /* commissioning failed */
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static esp_err_t zb_window_covering_handler(const esp_zb_zcl_window_covering_movement_message_t *message)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    switch (message->command) {
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN:
        ESP_LOGI(TAG, "Open up\n");
        start_move_task(&stepper_motor_driver, 0);
        break;
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE:
        ESP_LOGI(TAG, "Down close\n");
        start_move_task(&stepper_motor_driver, 100);
        break;
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_STOP:
        ESP_LOGI(TAG, "Stop\n");
        stop_move_task(&stepper_motor_driver);
        break;
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE:
        ESP_LOGI(TAG, "Go to lift percentage: %d\n", message->payload.percentage_lift_value);
        start_move_task(&stepper_motor_driver, message->payload.percentage_lift_value);
        break;
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_VALUE:
        ESP_LOGI(TAG, "Go to lift value: %d\n", message->payload.lift_value);
        break;
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_TILT_PERCENTAGE:
        ESP_LOGI(TAG, "Go to tilt percentage not implemented.\n");
        break;
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_TILT_VALUE:
        ESP_LOGI(TAG, "Go to tilt value not implemented.\n");
        break;
    default:
        break;
    }

    return ret;
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    if (message->info.dst_endpoint == HA_WINDOW_COVERING_ENDPOINT)
    {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING)
        {
            ESP_LOGI(TAG, "Window covering cluster, attribute ID: 0x%x", message->attribute.id);
        }
    }

    return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    case ESP_ZB_CORE_WINDOW_COVERING_MOVEMENT_CB_ID:
        ret = zb_window_covering_handler((esp_zb_zcl_window_covering_movement_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

static void esp_zb_task(void *pvParameters)
{
    /* Initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    /* Create customized window covering endpoint */
    esp_zb_ep_list_t *esp_zb_window_covering_ep = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_WINDOW_COVERING_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_WINDOW_COVERING_DEVICE_ID,
        .app_device_version = 0,
    };

    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    /* Create basic cluster */
    esp_zb_attribute_list_t *basic_attr_list = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    esp_zb_basic_cluster_add_attr(basic_attr_list, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, modelid);
    esp_zb_basic_cluster_add_attr(basic_attr_list, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, manufname);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    /* Create window covering cluster */
    esp_zb_attribute_list_t *window_attr_list = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING);
    esp_zb_window_covering_cluster_cfg_t config = {
        .covering_type = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_TYPE_ROLLERSHADE,
        .covering_status = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_OPERATIONAL,
        .covering_mode = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_TYPE_MOTOR_IS_RUNNING_IN_MAINTENANCE_MODE,
    };
    esp_zb_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_WINDOW_COVERING_TYPE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &config.covering_type);
    esp_zb_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_STATUS_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BITMAP, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &config.covering_status);
    esp_zb_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_MODE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BITMAP, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &config.covering_mode);

    uint8_t current_lift_percentage = 0x00;
    uint16_t installed_open_limit_lift = 0x0000;
    uint16_t installed_closed_limit_lift = 0xffff;
    uint16_t current_position_lift = ESP_ZB_ZCL_WINDOW_COVERING_CURRENT_POSITION_LIFT_DEFAULT_VALUE;

    esp_zb_window_covering_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID, &current_lift_percentage);
    esp_zb_window_covering_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_INSTALLED_OPEN_LIMIT_LIFT_ID, &installed_open_limit_lift);
    esp_zb_window_covering_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_INSTALLED_CLOSED_LIMIT_LIFT_ID, &installed_closed_limit_lift);
    esp_zb_window_covering_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_ID, &current_position_lift);

    esp_zb_cluster_list_add_window_covering_cluster(cluster_list, window_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_ep_list_add_ep(esp_zb_window_covering_ep, cluster_list, endpoint_config);
    esp_zb_device_register(esp_zb_window_covering_ep);

    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_main_loop_iteration();
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
