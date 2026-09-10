idf_component_register(SRCS "n10_bridge_main.c"
                       PRIV_REQUIRES esp_driver_uart esp_event esp_netif esp_wifi lwip nvs_flash
                       INCLUDE_DIRS "")
