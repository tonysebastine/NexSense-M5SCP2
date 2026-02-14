idf_component_register(
    SRCS "event_mgr.c"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash esp_timer
)
