idf_component_register(
    SRCS "feature_extraction.c"
    INCLUDE_DIRS "include"
    REQUIRES ds18b20_driver max30102_driver mpu6500_driver eda_driver
    PRIV_REQUIRES esp_timer
)
