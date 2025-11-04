# Component CMakeLists.txt for sensor_system
idf_component_register(SRCS "src/system_init.c"
                            "src/watchdog.c"
                            "src/error_recovery.c"
                            "src/wifi_manager.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_hw_support driver esp_netif esp_event esp_wifi nvs_flash)
