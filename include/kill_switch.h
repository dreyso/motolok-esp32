 #include "esp_gatts_api.h"
 
#define KS_APP_ID                   0x02
#define KS_SERVICE_APP_IDX          1
#define KS_SERVICE_INST_ID          1

#define GPIO_PIN                    2

 // Kill Switch attributes
 enum
 {
     KS_IDX_SVC,
 
     KS_IDX_OVERRIDE_CHAR,
     KS_IDX_OVERRIDE_VAL,

     KS_IDX_NB,
 };

 // Override States
 enum
{
    KS_OVERRIDE_OFF,    
    KS_OVERRIDE_0,    
    KS_OVERRIDE_1, 
    KS_OVERRIDE_NB           
};

void kill_switch_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
