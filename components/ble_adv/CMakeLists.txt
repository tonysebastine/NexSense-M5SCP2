idf_component_register(
    SRCS "ble_adv.c"
    INCLUDE_DIRS "include"
    REQUIRES bt event_mgr esp_timer mbedtls
)
