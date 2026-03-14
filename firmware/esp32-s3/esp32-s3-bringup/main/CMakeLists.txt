idf_component_register(
    SRCS "blink_example_main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver esp_driver_ledc esp_timer
)
