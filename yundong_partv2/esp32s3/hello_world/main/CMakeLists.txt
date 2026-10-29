idf_component_register(SRCS "start_bridge.c"
    INCLUDE_DIRS "../../../shared"
    PRIV_REQUIRES esp_driver_uart esp_wifi esp_event esp_netif nvs_flash lwip)
