#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

#include "basic.h"
#include "kill_switch.h"
#include "to_str.h"
 
#define TAG "Main"

#define MOTOLOK_SERVICE_NUM                       2

#define ADV_CONFIG_FLAG                           (1 << 0)
#define SCAN_RSP_CONFIG_FLAG                      (1 << 1)

static char device_name[ESP_BLE_ADV_DATA_LEN_MAX] = "Motolok";
static uint8_t adv_config_done = 0;

static uint8_t manufacturer[4]={'D', '2', '3', '7'};

static uint8_t sec_service_uuid[16] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 
    0x00, 0x10, 0x00, 0x00, 0x18, 0x0D, 0x00, 0x00,
};
 
static esp_ble_adv_data_t adv_config = {
    .set_scan_rsp = false,
    .include_txpower = true,
    .min_interval = 0x0006, 
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0, 
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(sec_service_uuid),
    .p_service_uuid = sec_service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t scan_rsp_config = {
    .set_scan_rsp = true,
    .include_name = true,
    .manufacturer_len = sizeof(manufacturer),
    .p_manufacturer_data = manufacturer,
};
 
static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x100,
    .adv_int_max        = 0x100,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_RPA_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};
 
struct gatts_service_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};
 
// Tie app, services,One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT 
static struct gatts_service_inst motolok_service_tab[MOTOLOK_SERVICE_NUM] = {
    [KS_SERVICE_APP_IDX] = {
        .gatts_cb = kill_switch_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       
    },
    [BASIC_SERVICE_APP_IDX] = {
        .gatts_cb = basic_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       
    },
};
 
static void show_bonded_devices(void)
{
    int dev_num = esp_ble_get_bond_device_num();
    if (dev_num == 0) {
        ESP_LOGI(TAG, "No bonded devices");
        return;
    }

    esp_ble_bond_dev_t *dev_list = malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    if (!dev_list) {
        ESP_LOGE(TAG, "Failed to allocate memory for bonded devices");
        return;
    }

    esp_ble_get_bond_device_list(&dev_num, dev_list);
    ESP_LOGI(TAG, "Bonded devices: %d", dev_num);

    for (int i = 0; i < dev_num; i++) {
        ESP_LOGI(TAG, "[%d] addr " ESP_BD_ADDR_STR, i, ESP_BD_ADDR_HEX(dev_list[i].bd_addr));
    }

    free(dev_list);
}

static void __attribute__((unused)) remove_all_bonded_devices(void)
{
    int dev_num = esp_ble_get_bond_device_num();
    if (dev_num == 0) {
        ESP_LOGI(TAG, "Bonded devices number zero\n");
        return;
    }

    esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    if (!dev_list) {
        ESP_LOGI(TAG, "malloc failed, return\n");
        return;
    }
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    for (int i = 0; i < dev_num; i++) {
        esp_ble_remove_bond_device(dev_list[i].bd_addr);
    }

    free(dev_list);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    ESP_LOGV(TAG, "GAP_EVT, event %d", event);

    switch (event) {
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising start failed, status %x", param->adv_start_cmpl.status);
                break;
            }
            ESP_LOGI(TAG, "Advertising start successfully");
            break;
        case ESP_GAP_BLE_LOCAL_IR_EVT:                               
            ESP_LOGI(TAG, "Local identity root");
            break;
        case ESP_GAP_BLE_LOCAL_ER_EVT:                               
            ESP_LOGI(TAG, "Local encryption root");
            break;
        case ESP_GAP_BLE_SEC_REQ_EVT:
            esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
            break;
        case ESP_GAP_BLE_KEY_EVT:
            ESP_LOGI(TAG, "Key exchanged, key_type %s", esp_key_type_to_str(param->ble_security.ble_key.key_type));
            break;
        case ESP_GAP_BLE_AUTH_CMPL_EVT:
            esp_bd_addr_t bd_addr;
            memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
            ESP_LOGI(TAG, "Authentication complete, addr_type %u, addr "ESP_BD_ADDR_STR"",
                    param->ble_security.auth_cmpl.addr_type, ESP_BD_ADDR_HEX(bd_addr));
                    
            if(!param->ble_security.auth_cmpl.success)
                ESP_LOGI(TAG, "Pairing failed, reason 0x%x",param->ble_security.auth_cmpl.fail_reason);
            else 
                ESP_LOGI(TAG, "Pairing successfully, auth_mode %s",esp_auth_req_to_str(param->ble_security.auth_cmpl.auth_mode));
            
            show_bonded_devices();
            break;
        case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT: {
            ESP_LOGD(TAG, "Bond device remove, status %d, device "ESP_BD_ADDR_STR"",
                    param->remove_bond_dev_cmpl.status, ESP_BD_ADDR_HEX(param->remove_bond_dev_cmpl.bd_addr));
            break;
        }
        case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
            if (param->local_privacy_cmpl.status != ESP_BT_STATUS_SUCCESS){
                ESP_LOGE(TAG, "Local privacy config failed, status %x", param->local_privacy_cmpl.status);
                break;
            }
            ESP_LOGI(TAG, "Local privacy config successfully");

            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_config);
            if (ret){
                ESP_LOGE(TAG, "config adv data failed, error code = %x", ret);
            }else{
                adv_config_done |= ADV_CONFIG_FLAG;
            }

            ret = esp_ble_gap_config_adv_data(&scan_rsp_config);
            if (ret){
                ESP_LOGE(TAG, "config adv data failed, error code = %x", ret);
            }else{
                adv_config_done |= SCAN_RSP_CONFIG_FLAG;
            }

            break;
        default:
            break;
    }
}

static bool is_bonded_device(esp_bd_addr_t remote_bda)
{
    int dev_num = esp_ble_get_bond_device_num();
    if (dev_num == 0) return false;

    esp_ble_bond_dev_t *dev_list = malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    if (!dev_list) {
        ESP_LOGE(TAG, "Malloc failed\n");
        return false;
    }

    if (esp_ble_get_bond_device_list(&dev_num, dev_list) != ESP_OK) {
        ESP_LOGE(TAG, "BLE Error");
        return false;
    }

    bool bonded = memcmp(dev_list[0].bd_addr, remote_bda, ESP_BD_ADDR_LEN) == 0;
    free(dev_list);
    return bonded;
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:
            if (param->reg.status == ESP_GATT_OK) {
                if (param->reg.app_id == BASIC_APP_ID) {
                    motolok_service_tab[BASIC_SERVICE_APP_IDX].gatts_if = gatts_if;
                    ESP_LOGI(TAG, "Basic service registered, gatt_if: %d", gatts_if);
                    
                    // Perform GAP settings only for one of the apps (e.g., Kill Switch)
                    esp_ble_gap_set_device_name(device_name);
                    esp_ble_gap_config_local_privacy(true);  // Generate a resolvable random address
                } else if (param->reg.app_id == KS_APP_ID) {
                    motolok_service_tab[KS_SERVICE_APP_IDX].gatts_if = gatts_if;
                    ESP_LOGI(TAG, "Kill Switch service registered, gatt_if: %d", gatts_if);
                } else {
                    ESP_LOGW(TAG, "Unknown app_id %04x, ignoring", param->reg.app_id);
                    return;
                }
            } else {
                ESP_LOGE(TAG, "Reg app failed, app_id %04x, status %d",
                        param->reg.app_id,
                        param->reg.status);
                return;
            }
            break;
        //--------------- Device specfic events will only be dispatched when caused by the bonded device
        case ESP_GATTS_CONNECT_EVT: 
            // Check if the connected device is bonded
            if (esp_ble_get_bond_device_num() > 0 && !is_bonded_device(param->connect.remote_bda)) {
                ESP_LOGE(TAG, "Disconnecting from non-bonded device: "ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(param->connect.remote_bda));
                esp_ble_gap_disconnect(param->connect.remote_bda);
                return;
            }
            
            // Keep connection and bond (or resume existing bond)
            ESP_LOGI(TAG, "Connected, conn_id %u, remote "ESP_BD_ADDR_STR"", param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));
            esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "Disconnected, remote "ESP_BD_ADDR_STR", reason 0x%x", ESP_BD_ADDR_HEX(param->disconnect.remote_bda), param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            
            if (!is_bonded_device(param->disconnect.remote_bda)) return;
            break;
        
        default:
            break;
    }

    // Dispatch service handlers
    for (int idx = 0; idx < MOTOLOK_SERVICE_NUM; idx++) {
        // Call all registered service callbacks
        if (gatts_if == ESP_GATT_IF_NONE || gatts_if == motolok_service_tab[idx].gatts_if) {
            if (motolok_service_tab[idx].gatts_cb) {
                motolok_service_tab[idx].gatts_cb(event, gatts_if, param);
            }
        }
    }
}

void app_main(void)
{
    esp_err_t ret;

    // Init NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s init controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "%s init bluetooth", __func__);

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(TAG, "gap register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(BASIC_APP_ID);
    if (ret){
        ESP_LOGE(TAG, "gatts app register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(KS_APP_ID);
    if (ret){
        ESP_LOGE(TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    // Set the security parameters
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_BOND;     

    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;          
    uint8_t key_size = 16;      
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_ENABLE;
    uint8_t oob_support = ESP_BLE_OOB_DISABLE;

    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
}