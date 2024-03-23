idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "include"
                    REQUIRES oled_display sgp41 sht45 spi_flash esp_timer senseair_s8 pms5003)
