esp_err_t nvs_write(const char *namespace, const char *key, uint8_t value);

esp_err_t nvs_read(const char *namespace, const char *key, uint8_t *out_value);

