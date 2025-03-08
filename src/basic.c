#include <inttypes.h>      
#include <string.h>        

#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "basic.h"  

#define TAG "Basic"

static uint16_t basic_handle_table[BASIC_IDX_NB];

static const uint8_t basic_svc[16] = {
    0xA7, 0xDD, 0xD2, 0xAF, 0xB1, 0x52, 0x96, 0x65,  
    0xC9, 0x83, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00   
};

// Reset characteristic, write only
static const uint8_t reset_uuid[16] = {
    0xAE, 0x33, 0xC3, 0xA0, 0x20, 0xEA, 0xCA, 0x85,
    0x3F, 0x4D, 0x30, 0x56, 0xAD, 0x94, 0x6B, 0xF0
};
static const uint8_t reset[1] = {0x00};

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint8_t char_prop_write_only = ESP_GATT_CHAR_PROP_BIT_WRITE;

// Basic GATT Database
static const esp_gatts_attr_db_t basic_gatt_db[BASIC_IDX_NB] =
{
    // Basic Service Declaration
    [BASIC_IDX_SVC] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
    sizeof(uint16_t), sizeof(basic_svc), (uint8_t *)&basic_svc}},

    // Reset Characteristic Declaration (Write Only)
    [BASIC_IDX_RESET_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write_only}},

    // Reset Characteristic Value (Write Only Encrypted)
    [BASIC_IDX_RESET_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&reset_uuid, ESP_GATT_PERM_WRITE_ENCRYPTED,
    sizeof(uint8_t), sizeof(reset), (uint8_t *)reset}},
};

void basic_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGV(TAG, "event = %x",event);
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "GATT server register, status %d, app_id %d, gatts_if %d",
                    param->reg.status, param->reg.app_id, gatts_if);
            esp_ble_gatts_create_attr_tab(basic_gatt_db, gatts_if, BASIC_IDX_NB, BASIC_SERVICE_INST_ID);
            break;
        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(TAG, "Characteristic write, value ");
            ESP_LOG_BUFFER_HEX(TAG, param->write.value, param->write.len);

            esp_gatt_status_t status = ESP_GATT_OK;
                
            // Reset
            if (param->write.handle == basic_handle_table[BASIC_IDX_RESET_VAL]) {
                ESP_LOGI(TAG, "Reset triggered, clearing NVS and restarting...");

                // Initialize NVS
                esp_err_t err = nvs_flash_erase();
                if (err != ESP_OK) ESP_LOGE(TAG, "Failed to erase NVS: %s", esp_err_to_name(err));

                // Restart the ESP32
                esp_restart();
            }

            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
            if (param->create.status == ESP_GATT_OK){
                if(param->add_attr_tab.num_handle == BASIC_IDX_NB) {
                    ESP_LOGI(TAG, "Attribute table create successfully, num_handle %x", param->add_attr_tab.num_handle);
                    memcpy(basic_handle_table, param->add_attr_tab.handles,
                    sizeof(basic_handle_table));
                    esp_ble_gatts_start_service(basic_handle_table[BASIC_IDX_SVC]);
                }else{
                    ESP_LOGE(TAG, "Attribute table create abnormally, num_handle (%d) doesn't equal to BASIC_IDX_NB(%d)",
                        param->add_attr_tab.num_handle, BASIC_IDX_NB);
                }
            }else{
                ESP_LOGE(TAG, "Attribute table create failed, error code = %x", param->create.status);
            }
        break;
    }

        default:
        break;
    }
}
