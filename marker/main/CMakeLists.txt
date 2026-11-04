idf_component_register(SRCS "marker_main.c" "markercode.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_ledc esp_driver_gpio esp_timer console nvs_flash)
