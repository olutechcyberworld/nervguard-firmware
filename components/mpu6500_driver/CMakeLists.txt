idf_component_register(
    SRCS "mpu6500_driver.c"
    INCLUDE_DIRS "include"
    REQUIRES i2c_bus
)
