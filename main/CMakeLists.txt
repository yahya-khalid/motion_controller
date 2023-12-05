idf_component_register(
    SRCS "gyro_mouse.c" "mpu6886.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_adc
    )
