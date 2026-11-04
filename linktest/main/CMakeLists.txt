idf_component_register(SRCS "linktest_main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_wifi esp_netif esp_event esp_timer esp_driver_gpio console nvs_flash lwip)
