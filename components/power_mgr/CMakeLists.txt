idf_component_register(
    SRCS "power_mgr.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer esp_pm esp_adc
)
