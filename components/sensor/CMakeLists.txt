idf_component_register(
    SRCS "sensor.c"
    INCLUDE_DIRS "include"
    REQUIRES driver nvs_flash esp_timer
)
