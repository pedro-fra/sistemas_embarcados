idf_component_register(SRCS "main.c" "ssd1306.c"
                    PRIV_REQUIRES driver esp_timer freertos
                    INCLUDE_DIRS "")
