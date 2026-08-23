idf_component_register(
    SRCS "ble_gatt_server.c"
    INCLUDE_DIRS "include"
    REQUIRES feature_extraction
    PRIV_REQUIRES bt nvs_flash esp_timer
)

# "bt" is ESP-IDF's own built-in Bluetooth component — no
# idf_component.yml needed here, unlike the drivers earlier in this
# project. It just needs to be turned ON in sdkconfig first:
#   idf.py menuconfig  ->  Component config  ->  Bluetooth  ->  enable it
#   Then: Bluetooth Host  ->  select "NimBLE - BLE only"
# This CMakeLists.txt can't flip that setting for you — it's a project-
# level config choice, not something a component file controls.
