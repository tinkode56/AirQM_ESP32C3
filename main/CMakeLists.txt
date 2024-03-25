idf_component_register(SRCS "airqm_influxdb.c" "airqm_main.c" "airqm_wifi.c"
                    INCLUDE_DIRS "include"
                    REQUIRES oled_display sgp41 sht45 spi_flash esp_timer senseair_s8 pms5003 nvs_flash esp_wifi wifi_provisioning esp_http_client)
