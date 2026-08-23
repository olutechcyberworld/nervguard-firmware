# This file is what makes ESP-IDF (and PlatformIO's espidf framework,
# which builds on top of ESP-IDF directly) recognise this folder as a
# component: a self-contained unit of code with its own source files
# and its own dependencies, rather than just a folder full of C files.
#
# SRCS          — which .c files belong to this component
# INCLUDE_DIRS   — which folder holds the .h file other code will include
# REQUIRES       — which other components this one needs to compile,
#                   in this case the two small libraries this driver
#                   depends on for 1-Wire communication and DS18B20
#                   command handling (pulled in automatically by
#                   idf_component.yml, sitting alongside this file)

idf_component_register(
    SRCS "ds18b20_driver.c"
    INCLUDE_DIRS "include"
    REQUIRES onewire_bus ds18b20
    PRIV_REQUIRES esp_timer
)