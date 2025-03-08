#include "esp_gatts_api.h"

#define BASIC_APP_ID                              0x01
#define BASIC_SERVICE_APP_IDX                     0
#define BASIC_SERVICE_INST_ID                     0

 // Reset attributes
 enum
 {
    BASIC_IDX_SVC,
 
    BASIC_IDX_RESET_CHAR,
    BASIC_IDX_RESET_VAL,

    BASIC_IDX_NB,
 };

void basic_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

