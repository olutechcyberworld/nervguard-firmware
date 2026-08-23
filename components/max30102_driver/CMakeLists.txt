idf_component_register(
    SRCS "max30102_driver.c"
    INCLUDE_DIRS "include"
    REQUIRES i2c_bus
    PRIV_REQUIRES esp_timer
)