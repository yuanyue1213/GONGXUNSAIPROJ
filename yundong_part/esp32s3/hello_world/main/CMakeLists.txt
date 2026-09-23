idf_component_register(SRCS "robot_remote_main.c"
                       PRIV_REQUIRES esp_wifi esp_netif esp_event nvs_flash esp_timer lwip esp_driver_uart
                       INCLUDE_DIRS "")
