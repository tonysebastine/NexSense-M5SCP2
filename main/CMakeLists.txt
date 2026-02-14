idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES sensor state_engine event_mgr ble_adv power_mgr ui
             nvs_flash esp_timer driver
)
