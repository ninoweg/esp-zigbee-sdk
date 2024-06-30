/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee HA_on_off_light Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include "esp_zb_window_covering.h"
#include "stepper_motor_driver.h"

#include "string.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zboss_api.h"

#define ARRAY_LENTH(arr) (sizeof(arr) / sizeof(arr[0]))

#if defined ZB_ED_ROLE
#error Define ZB_COORDINATOR_ROLE in idf.py menuconfig to compile thermostat source code.
#endif

static const char *TAG = "ESP_ZB_WINDOW_COVERING";
static stepper_driver stepper_motor_driver;

char modelid[] = {24, 'E', 'S', 'P', '3', '2', '-', 'H', '2', '-', 'W', 'i', 'n', 'd', 'o', 'w', '-', 'C', 'o', 'v', 'e', 'r', 'i', 'n', 'g'};
char manufname[] = {9, 'E', 's', 'p', 'r', 'e', 's', 's', 'i', 'f'};

/********************* Define functions **************************/
static esp_err_t deferred_driver_init(void)
{
    /* Initialize stepper driver */
    init_stepper_driver(&stepper_motor_driver, 11, 25, 12, 8);
    set_rpm(&stepper_motor_driver, 12);
    return ESP_OK;
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
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

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    // bool light_state = 0;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    // if (message->info.dst_endpoint == HA_ESP_LIGHT_ENDPOINT) {
    //     if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
    //         if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
    //             light_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : light_state;
    //             ESP_LOGI(TAG, "Light sets to %s", light_state ? "On" : "Off");
    //             light_driver_set_power(light_state);
    //         }
    //     }
    // }
    return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

bool raw_command_handler(zb_uint8_t bufid)
{
    printf("raw cmd:\n");
    uint8_t buf[zb_buf_len(bufid)];
    zb_zcl_parsed_hdr_t *cmd_info = ZB_BUF_GET_PARAM(bufid, zb_zcl_parsed_hdr_t);
    printf("cluster_id: 0x%x \n", cmd_info->cluster_id);
    printf("profile_id: 0x%x \n", cmd_info->profile_id);
    printf("cmd_id: %d \n", cmd_info->cmd_id);
    printf("cmd_direction: %d \n", cmd_info->cmd_direction);
    printf("seq_number: %d \n", cmd_info->seq_number);
    printf("is_common_command: %d \n", cmd_info->is_common_command);
    printf("disable_default_response: %d \n", cmd_info->disable_default_response);
    printf("is_manuf_specific: %d \n", cmd_info->is_manuf_specific);
    printf("manuf_specific: %d \n", cmd_info->manuf_specific);
    memcpy(buf, zb_buf_begin(bufid), sizeof(buf));
    ESP_LOGI("RAW", "bufid: %d size: %d", bufid, sizeof(buf));
    for (int i = 0; i < sizeof(buf); ++i) {
        printf("0x%02X ", buf[i]);
    }
    printf("\n");

    // if (cmd_info->is_common_command)
    // {
    //     return true;
    // }

    if (cmd_info->cluster_id == ESP_ZB_ZCL_CLUSTER_ID_BASIC)
    {
        ESP_LOGW(TAG, "Basic Cluster\n\n");
        return false;
    }
    else if (cmd_info->cluster_id == ZB_ZCL_CLUSTER_ID_WINDOW_COVERING)
    {
        ESP_LOGW(TAG, "Window Covering\n\n");
        zb_uint8_t command_id = cmd_info->cmd_id;
        switch (command_id)
        {
        case ESP_ZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN:
            ESP_LOGW(TAG, "Open up\n");
            start_move_task(&stepper_motor_driver, true);
            return true;
            break;
        case ESP_ZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE:
            ESP_LOGW(TAG, "Down close\n");
            start_move_task(&stepper_motor_driver, false);
            return true;
            break;
        case ESP_ZB_ZCL_CMD_WINDOW_COVERING_STOP:
            ESP_LOGW(TAG, "Stop\n");
            stop_move_task(&stepper_motor_driver);
            return true;
            break;
        case ESP_ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE:
            ESP_LOGW(TAG, "Go to lift percentage\n");
            return true;
            break;
        case ESP_ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_VALUE:
            ESP_LOGW(TAG, "Go to tilt value\n");
            return true;
            break;
        case ESP_ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_TILT_PERCENTAGE:
            ESP_LOGW(TAG, "Go to tilt percentage\n");
            return true;
            break;
        case ESP_ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_TILT_VALUE:
            ESP_LOGW(TAG, "Go to tilt value\n");
            return true;
            break;
        default:
            return false;
            break;
        }
    }
    else 
    {
        ESP_LOGW(TAG, "Other\n\n");
        return false;
    }
    return false;
}

static esp_zb_cluster_list_t *custom_window_covering_clusters_create()
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    /* Set the NULL to esp_zb_window_covering_cluster_create() means that using the default attribute value for window covering cluster */
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, &modelid[0]);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, &manufname[0]);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_attribute_list_t *window_attr_list = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING);
    esp_zb_window_covering_cluster_cfg_t config = {
        .covering_type = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_TYPE_ROLLERSHADE,
        .covering_status = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_LIFT_CONTROL_IS_CLOSED_LOOP,
        .covering_mode = ESP_ZB_ZCL_WINDOW_COVERING_MODE_DEFAULT_VALUE,
    };

    esp_zb_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_WINDOW_COVERING_TYPE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &config.covering_type);
    esp_zb_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_STATUS_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BITMAP, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &config.covering_status);
    esp_zb_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_MODE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BITMAP, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &config.covering_mode);
    
    uint8_t current_tilt_percentage = 0;
    uint16_t installed_open_limit_tilt = 0x0000;
    uint16_t installed_closed_limit_tilt = 0xffff;
    uint16_t current_position_tilt = ESP_ZB_ZCL_WINDOW_COVERING_CURRENT_POSITION_TILT_DEFAULT_VALUE;

    esp_zb_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_TILT_PERCENTAGE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BIT, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &current_tilt_percentage);
    esp_zb_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_INSTALLED_OPEN_LIMIT_TILT_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_16BIT, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &installed_open_limit_tilt);
    esp_zb_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_INSTALLED_CLOSED_LIMIT_TILT_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_16BIT, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &installed_closed_limit_tilt);
    esp_zb_cluster_add_attr(window_attr_list, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_TILT_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_16BIT, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &current_position_tilt);
    esp_zb_cluster_list_add_window_covering_cluster(cluster_list, window_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    return cluster_list;
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
    esp_zb_ep_list_add_ep(esp_zb_window_covering_ep, custom_window_covering_clusters_create(), endpoint_config);

    /* Register the device */
    esp_zb_device_register(esp_zb_window_covering_ep);

    esp_zb_raw_command_handler_register(raw_command_handler);
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
