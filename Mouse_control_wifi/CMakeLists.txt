idf_component_register(SRCS "main.c" 
                       INCLUDE_DIRS "."
                       REQUIRES tinyusb esp_wifi esp_timer nvs_flash lwip driver
                       )