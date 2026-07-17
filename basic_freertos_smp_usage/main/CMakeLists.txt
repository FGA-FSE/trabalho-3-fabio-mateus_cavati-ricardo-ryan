idf_component_register(SRCS "esp_hidd_prf_api.c" "hid_device_le_prf.c" "hid_dev.c" "main.c"
                       PRIV_REQUIRES driver esp_adc esp_driver_gpio bt nvs_flash esp_hid
                       INCLUDE_DIRS "")