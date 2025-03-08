#include <inttypes.h>      
#include <string.h>        

#include "driver/gpio.h"   
#include "esp_log.h"       

#include "kill_switch.h"  

#define TAG "Kill Switch"

static uint16_t kill_switch_handle_table[KS_IDX_NB];
static uint8_t override = KS_OVERRIDE_OFF;

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
static const esp_gatts_attr_db_t kill_switch_gatt_db[KS_IDX_NB] =
{
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
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, (uint8_t *)&override_uuid, ESP_GATT_PERM_WRITE_ENCRYPTED | ESP_GATT_PERM_READ_ENCRYPTED,
    sizeof(uint8_t), sizeof(override_val), (uint8_t *)override_val}},
};

static esp_gatt_status_t handle_override_value(uint8_t written_value) {
    if(written_value >= KS_OVERRIDE_NB){
        ESP_LOGW(TAG, "Invalid override value: %d, rejecting", written_value);
        return ESP_GATT_WRITE_NOT_PERMIT;
    }
    
    switch (written_value) {
        case KS_OVERRIDE_OFF:
            ESP_LOGI(TAG, "Kill switch override: OFF");
            break;
        case KS_OVERRIDE_0:
            gpio_set_level(GPIO_PIN, 0);
            ESP_LOGI(TAG, "Kill switch override: 0");
            break;
        case KS_OVERRIDE_1:
            gpio_set_level(GPIO_PIN, 1);
            ESP_LOGI(TAG, "Kill switch override: 1");
            break;
        }
        
    override = written_value;
    return ESP_GATT_OK;
}

void kill_switch_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGV(TAG, "event = %x",event);
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "GATT server register, status %d, app_id %d, gatts_if %d",
                    param->reg.status, param->reg.app_id, gatts_if);

            // Init GPIO pin
            esp_rom_gpio_pad_select_gpio(GPIO_PIN);
            gpio_set_direction(GPIO_PIN, GPIO_MODE_OUTPUT);
            gpio_set_level(GPIO_PIN, 0);
              
            esp_ble_gatts_create_attr_tab(kill_switch_gatt_db, gatts_if, KS_IDX_NB, KS_SERVICE_INST_ID);
            break;
        case ESP_GATTS_CONNECT_EVT:
            if (override != KS_OVERRIDE_OFF)
                return;;

            gpio_set_level(GPIO_PIN, 1);
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            if (override != KS_OVERRIDE_OFF)
                return;

            gpio_set_level(GPIO_PIN, 0);
            break;
        case ESP_GATTS_READ_EVT:
            break;
        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(TAG, "Characteristic write, value ");
            ESP_LOG_BUFFER_HEX(TAG, param->write.value, param->write.len);
            
            esp_gatt_status_t status = ESP_GATT_OK;
                
            // Override
            if (param->write.handle == kill_switch_handle_table[KS_IDX_OVERRIDE_VAL]) {
                if (param->write.len == 1) {
                    status = handle_override_value(param->write.value[0]);
                } else {
                    status = ESP_GATT_INVALID_ATTR_LEN;
                    ESP_LOGW(TAG, "Invalid write length (%d), rejecting", param->write.len);
                }
            }

            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
            if (param->create.status == ESP_GATT_OK){
                if(param->add_attr_tab.num_handle == KS_IDX_NB) {
                    ESP_LOGI(TAG, "Attribute table create successfully, num_handle %x", param->add_attr_tab.num_handle);
                    memcpy(kill_switch_handle_table, param->add_attr_tab.handles,
                    sizeof(kill_switch_handle_table));
                    esp_ble_gatts_start_service(kill_switch_handle_table[KS_IDX_SVC]);
                }else{
                    ESP_LOGE(TAG, "Attribute table create abnormally, num_handle (%d) doesn't equal to KS_IDX_NB(%d)",
                        param->add_attr_tab.num_handle, KS_IDX_NB);
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