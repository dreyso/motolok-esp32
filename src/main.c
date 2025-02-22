#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#define TAG "Motolok"
#define GATTS_SERVICE_UUID    0x00FF
#define GATTS_CHAR_UUID       0xFF01
#define GATTS_NUM_HANDLES     4
#define SWITCH_GPIO           2  // Change this to your actual switch GPIO pin

static uint16_t service_handle;
static uint16_t char_handle;

// BLE Advertisement Parameters
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,  // 20ms
    .adv_int_max = 0x30,  // 30ms
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// BLE Advertisement Data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .flag = ESP_BLE_ADV_FLAG_LIMIT_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT
};

// BLE Connection Parameters
esp_ble_conn_update_params_t conn_params = {
    .min_int = 0x630,  // 1584 * 1.25ms = 1980ms
    .max_int = 0x640,  // 1600 * 1.25ms = 2000ms
    .latency = 0,
    .timeout = 500,    // 500 * 10ms = 5000ms
    .bda = {0}         // Placeholder for peer device's Bluetooth address
};

// FreeRTOS Timer Handle
static TimerHandle_t adv_timer;

void stop_advertising_cb(TimerHandle_t xTimer) {
    ESP_LOGI(TAG, "Discovery timed out.");
    esp_ble_gap_stop_advertising();
}

void stop_advertising() {
    xTimerStop(adv_timer, 0);  // Stop the timer to prevent further callbacks
    esp_ble_gap_stop_advertising();
}

void start_advertising() {
    esp_ble_gap_start_advertising(&adv_params);
    xTimerStart(adv_timer, 0);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_OK) {
                ESP_LOGI(TAG, "Advertising started.");
            } else {
                ESP_LOGE(TAG, "Advertising failed to start: %d", param->adv_start_cmpl.status);
            }
            break;

        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status == ESP_OK) {
                ESP_LOGI(TAG, "Advertising stopped.");
            } else {
                ESP_LOGE(TAG, "Advertising failed to stop: %d", param->adv_stop_cmpl.status);
            }
            break;

        default:
            break;
    }
}

// GATTS Event Handler
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "Registering service...");
            esp_ble_gatts_create_service(gatts_if, &(esp_gatt_srvc_id_t){
                .id = {.uuid = {.len = ESP_UUID_LEN_16, .uuid = {.uuid16 = GATTS_SERVICE_UUID}}},
                .is_primary = true
            }, GATTS_NUM_HANDLES);
            break;

        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(TAG, "Service created, starting service...");
            service_handle = param->create.service_handle;
            esp_ble_gatts_start_service(service_handle);

            ESP_LOGI(TAG, "Adding writable characteristic...");
            esp_ble_gatts_add_char(service_handle, &(esp_bt_uuid_t){
                .len = ESP_UUID_LEN_16, .uuid = {.uuid16 = GATTS_CHAR_UUID}
            }, ESP_GATT_PERM_WRITE, ESP_GATT_CHAR_PROP_BIT_WRITE, NULL, NULL);
            break;

        case ESP_GATTS_ADD_CHAR_EVT:
            char_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "Characteristic added, handle: %d", char_handle);
            break;

        case ESP_GATTS_WRITE_EVT:
            if (param->write.handle == char_handle) {
                uint8_t received_value = param->write.value[0];
                ESP_LOGI(TAG, "Received Write: %d", received_value);

                if (received_value == 1) {
                    gpio_set_level(SWITCH_GPIO, 1);  
                    ESP_LOGI(TAG, "Switch turned ON");
                } else if (received_value == 0) {
                    gpio_set_level(SWITCH_GPIO, 0); 
                    ESP_LOGI(TAG, "Switch turned OFF");
                }
            }
            break;

        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "Device connected.");

            stop_advertising();

            // Copy peer device address to connection parameters
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(conn_params.bda));

            // Set connection params
            ESP_ERROR_CHECK(esp_ble_gap_update_conn_params(&conn_params));
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "Device disconnected.");
            
            start_advertising();
            break;

        default:
            break;
    }
}

void app_main() {
    ESP_LOGI(TAG, "Initializing BLE...");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Init timer
    adv_timer = xTimerCreate("AdvTimer", pdMS_TO_TICKS(30000), pdFALSE, (void *)0, stop_advertising_cb);
    if (adv_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create timer.");
        return;
    }

    // Initialize Bluetooth
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    // Initialize Bluedroid stack
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // Set the device name
    const char *device_name = "Motolok";
    ESP_ERROR_CHECK(esp_ble_gap_set_device_name(device_name));

    // Register GAP and GATTS callbacks
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));

    // Configure advertisement data
    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));

    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_N0);
    
    start_advertising();

    // Initialize GPIO for switch control
    gpio_reset_pin(SWITCH_GPIO);  // Reset GPIO before configuring
    gpio_set_direction(SWITCH_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(SWITCH_GPIO, 0);  // Default to OFF

    ESP_LOGI(TAG, "BLE server initialized.");
}
