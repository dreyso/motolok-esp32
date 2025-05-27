#include "nvs.h"
#include "nvs_handle.h"

esp_err_t nvs_write(const char *namespace, const char *key, uint8_t value) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(namespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    err = nvs_set_u8(handle, key, value);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }

    nvs_close(handle);
    return err;
}

esp_err_t nvs_read(const char *namespace, const char *key, uint8_t *out_value) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(namespace, NVS_READONLY, &handle);
    if (err != ESP_OK) return err;

    err = nvs_get_u8(handle, key, out_value);
    nvs_close(handle);
    return err;
}

