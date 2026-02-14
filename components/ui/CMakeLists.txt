idf_component_register(
    SRCS "ui.c"
    INCLUDE_DIRS "include"
    REQUIRES driver state_engine power_mgr esp_timer
)
