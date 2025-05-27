#include <inttypes.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"

#include "kill_switch.h"
#include "nvs_handle.h"

#define TAG "Kill Switch"

static uint16_t kill_switch_handle_table[KS_IDX_NB];
static uint8_t override = KS_OVERRIDE_OFF;
static bool device_connected = false;

// Kill Switch Service
static const uint8_t kill_switch_svc[16] = {
    0x26, 0xa0, 0x47, 0xd6, 0x87, 0x7c, 0xdf, 0x83,
    0x8f, 0x42, 0xe5, 0x76, 0x75, 0x4f, 0x59, 0xf3
};

// Override characteristic, read & write
static const uint8_t override_uuid[16] = {
    0x41, 0x61, 0xba, 0xa7, 0xa5, 0xac, 0xa2, 0xb0,
    0x77, 0x42, 0x31, 0xcb, 0x6f, 0xcd, 0x54, 0x3f
};

static const uint8_t override_val[1] = {0x00};

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ;

// Kill Switch GATT Database
static const esp_gatts_attr_db_t kill_switch_gatt_db[KS_IDX_NB] = {
    // Kill Switch Service Declaration
    [KS_IDX_SVC] = 
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
        sizeof(uint16_t), sizeof(kill_switch_svc), (uint8_t *)&kill_switch_svc}},

    // Override Characteristic Declaration (Read & Write)
    [KS_IDX_OVERRIDE_CHAR] = 
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    // Override Characteristic Value (Read & Write Encrypted)
    [KS_IDX_OVERRIDE_VAL] = 
        {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, (uint8_t *)&override_uuid, ESP_GATT_PERM_WRITE | ESP_GATT_PERM_READ,
        sizeof(uint8_t), sizeof(override_val), (uint8_t *)override_val}},
};

static void update_pin() {
    switch (override) {
        case KS_OVERRIDE_OFF:
            ESP_LOGI(TAG, "Kill switch override: OFF");
            gpio_set_level(GPIO_PIN, device_connected ? 1 : 0);
            break;
        case KS_OVERRIDE_0:
            ESP_LOGI(TAG, "Kill switch override: 0");
            gpio_set_level(GPIO_PIN, 0);
            break;
        case KS_OVERRIDE_1:
            ESP_LOGI(TAG, "Kill switch override: 1");
            gpio_set_level(GPIO_PIN, 1);
            break;
    }
}

static esp_gatt_status_t handle_override_value(uint8_t written_value) {
    if (written_value >= KS_OVERRIDE_NB) {
        ESP_LOGW(TAG, "Invalid override value: %d, rejecting", written_value);
        return ESP_GATT_WRITE_NOT_PERMIT;
    }

    override = written_value;
    update_pin();
    return ESP_GATT_OK;
}

void kill_switch_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    ESP_LOGV(TAG, "event = %x", event);

    switch (event) {
        case ESP_GATTS_REG_EVT: {
            ESP_LOGI(TAG, "GATT server register, status %d, app_id %d, gatts_if %d",
                     param->reg.status, param->reg.app_id, gatts_if);

            // Load in override value
            esp_err_t err = nvs_read("kill_switch", "override", &override);
            if (err == ESP_OK) 
                ESP_LOGI(TAG, "Loaded override: %d", override);
            else 
                ESP_LOGI(TAG, "Failed to read override: %s", esp_err_to_name(err));
            

            // Init GPIO pin
            esp_rom_gpio_pad_select_gpio(GPIO_PIN);
            gpio_set_direction(GPIO_PIN, GPIO_MODE_OUTPUT);
            update_pin();

            esp_ble_gatts_create_attr_tab(kill_switch_gatt_db, gatts_if, KS_IDX_NB, KS_SERVICE_INST_ID);
            break;
        }

        case ESP_GATTS_CONNECT_EVT:
            device_connected = true;
            update_pin();
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            device_connected = false;
            update_pin();
            break;

        case ESP_GATTS_READ_EVT: {
            esp_gatt_rsp_t rsp = {};
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = 1;
            rsp.attr_value.value[0] = override_val[0];

            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
            break;
        }

        case ESP_GATTS_WRITE_EVT: {
            ESP_LOGI(TAG, "Characteristic write, value ");
            ESP_LOG_BUFFER_HEX(TAG, param->write.value, param->write.len);

            esp_gatt_status_t status = ESP_GATT_OK;

            if (param->write.handle == kill_switch_handle_table[KS_IDX_OVERRIDE_VAL]) {
                if (param->write.len == 1)
                    status = handle_override_value(param->write.value[0]);
                else {
                    status = ESP_GATT_INVALID_ATTR_LEN;
                    ESP_LOGW(TAG, "Invalid write length (%d), rejecting", param->write.len);
                }
            }

            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);

            // Store value in NVS
            esp_err_t err = nvs_write("kill_switch", "override", override);
            if (err != ESP_OK)
                ESP_LOGE(TAG, "Failed to write override: %s", esp_err_to_name(err));
            break;
        }

        case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
            if (param->create.status == ESP_GATT_OK) {
                if (param->add_attr_tab.num_handle == KS_IDX_NB) {
                    ESP_LOGI(TAG, "Attribute table created successfully, num_handle %x", param->add_attr_tab.num_handle);
                    memcpy(kill_switch_handle_table, param->add_attr_tab.handles, sizeof(kill_switch_handle_table));
                    esp_ble_gatts_start_service(kill_switch_handle_table[KS_IDX_SVC]);
                } else 
                    ESP_LOGE(TAG, "Attribute table creation mismatch: num_handle (%d) != KS_IDX_NB(%d)", param->add_attr_tab.num_handle, KS_IDX_NB);
            } else
                ESP_LOGE(TAG, "Attribute table creation failed, error code = %x", param->create.status);
            break;
        }

        default:
            break;
    }
}
