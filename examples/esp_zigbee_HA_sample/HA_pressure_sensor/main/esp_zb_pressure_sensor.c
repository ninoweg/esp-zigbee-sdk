/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee HA_temperature_sensor Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include "esp_zb_pressure_sensor.h"
#include "pressure_sensor_driver.h"

#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile sensor (End Device) source code.
#endif

static const char *TAG = "ESP_ZB_TEMP_SENSOR";

static int16_t zb_pressure_to_s16(float temp)
{
    return (int16_t)(temp);
}

static void esp_app_pressure_sensor_handler(float pressure)
{
    int16_t measured_value = zb_pressure_to_s16(pressure);
    /* Update pressure sensor measured value */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(HA_PRESSURE_MEASUREMENT_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT, 
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID, 
                                 &measured_value, 
                                 false);
    esp_zb_lock_release();

    esp_zb_zcl_report_attr_cmd_t report_attr_cmd;
    report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
    report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID;
    report_attr_cmd.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
    report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT;
    report_attr_cmd.zcl_basic_cmd.src_endpoint = HA_PRESSURE_MEASUREMENT_ENDPOINT;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
    esp_zb_lock_release();
    ESP_EARLY_LOGI(TAG, "Send 'report attributes' command");
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, ,
                        TAG, "Failed to start Zigbee bdb commissioning");
}

static esp_err_t deferred_driver_init(void)
{
    ESP_RETURN_ON_ERROR(pressure_sensor_driver_init(ESP_PRESSURE_MEASUREMENT_UPDATE_INTERVAL, esp_app_pressure_sensor_handler), TAG, "Failed to initialize temperature sensor");
    return ESP_OK;
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p     = signal_struct->p_app_signal;
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

static void esp_zb_task(void *pvParameters)
{
    /* Initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_PRESSURE_MEASUREMENT_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
        .app_device_version = 0
    };

    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();

    /* Create basic cluster */
    esp_zb_attribute_list_t *basic_attr_list = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_attr_list, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_attr_list, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, basic_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    /* Create Identify cluster */
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    // ------------------------------ Cluster Pressure ------------------------------
    esp_zb_pressure_meas_cluster_cfg_t pressure_meas_cfg = {
        .measured_value = 0x8000,
        .min_value = 200,
        .max_value = 1300,
    };
    // uint16_t pressure_tolerance = (CONFIG_PRJ_PRESSURE_SENSOR_ABSOLUTE_PRECISION / 100.0 + 0.5);
    int16_t pressure_scaled_value = 0x8000; // unit: 0.1hPa, scale=1
    int16_t pressure_value = 0x8000; 
    int16_t pressure_min_scaled_value = 2000;
    int16_t pressure_max_scaled_value = 13000;
    uint8_t pressure_scale = 1;
    esp_zb_attribute_list_t *esp_zb_pressure_meas_cluster = esp_zb_pressure_meas_cluster_create(&pressure_meas_cfg);
    // ESP_ERROR_CHECK(esp_zb_pressure_meas_cluster_add_attr(esp_zb_pressure_meas_cluster, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_TOLERANCE_ID, &pressure_tolerance));  // not supported
    ESP_ERROR_CHECK(esp_zb_pressure_meas_cluster_add_attr(esp_zb_pressure_meas_cluster, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_SCALED_VALUE_ID, &pressure_scaled_value));
    ESP_ERROR_CHECK(esp_zb_pressure_meas_cluster_add_attr(esp_zb_pressure_meas_cluster, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_MIN_SCALED_VALUE_ID, &pressure_min_scaled_value));
    ESP_ERROR_CHECK(esp_zb_pressure_meas_cluster_add_attr(esp_zb_pressure_meas_cluster, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_MAX_SCALED_VALUE_ID, &pressure_max_scaled_value));
    ESP_ERROR_CHECK(esp_zb_pressure_meas_cluster_add_attr(esp_zb_pressure_meas_cluster, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_SCALE_ID, &pressure_scale));
    esp_zb_cluster_list_add_pressure_meas_cluster(esp_zb_cluster_list, esp_zb_pressure_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* Register the device */
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_config);
    esp_zb_device_register(esp_zb_ep_list);

    /* Config the reporting info  */
    esp_zb_zcl_reporting_info_t reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_PRESSURE_MEASUREMENT_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,
        .u.send_info.max_interval = 0,
        .u.send_info.def_min_interval = 1,
        .u.send_info.def_max_interval = 0,
        .u.send_info.delta.u16 = 100,
        .attr_id = ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };

    esp_zb_zcl_update_reporting_info(&reporting_info);

    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));

    esp_zb_stack_main_loop();
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    /* Start Zigbee stack task */
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
