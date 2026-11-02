idf_component_register(SRCS "../../../yundong_partv2/esp32s3/hello_world/main/start_bridge.c"
                       PRIV_REQUIRES esp_driver_uart esp_event esp_netif esp_wifi esp_timer lwip nvs_flash
                       INCLUDE_DIRS "../../../yundong_partv2/shared")
