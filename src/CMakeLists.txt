# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

# REQUIRES was added here: main.c now includes ds18b20_driver.h, so this
# needs to explicitly say it depends on that component, the same rule
# that made the driver itself need PRIV_REQUIRES esp_timer earlier.
# Every new sensor driver added later gets appended to this same list.
idf_component_register(SRCS ${app_sources} REQUIRES ds18b20_driver max30102_driver mpu6500_driver eda_driver vibration_driver rgb_led_driver feature_extraction tflite_inference ble_gatt_server nvs_flash)