#include "i2c_bus.h"
#include "esp_log.h"

// GPIO8 (SDA) and GPIO9 (SCL), locked in circuit_design_handoff.md, do
// not change without a formal circuit revision. Physical 4.7kΩ pull-up
// resistors (R2, R3) are already on the board, so this driver does not
// enable the ESP32-S3's own internal pull-ups as well.
#define I2C_BUS_SDA_GPIO 8
#define I2C_BUS_SCL_GPIO 9
#define I2C_BUS_PORT     I2C_NUM_0

static const char *TAG = "i2c_bus";
static i2c_master_bus_handle_t s_bus_handle = NULL;

esp_err_t i2c_bus_init(void)
{
    if (s_bus_handle != NULL) {
        // Already set up by an earlier call (e.g. from another driver
        // that also depends on this bus). Nothing more to do.
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_BUS_PORT,
        .sda_io_num = I2C_BUS_SDA_GPIO,
        .scl_io_num = I2C_BUS_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &s_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set up the shared I2C bus (SDA=GPIO%d, SCL=GPIO%d): %s",
                  I2C_BUS_SDA_GPIO, I2C_BUS_SCL_GPIO, esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Shared I2C bus ready: SDA=GPIO%d, SCL=GPIO%d", I2C_BUS_SDA_GPIO, I2C_BUS_SCL_GPIO);
    return ESP_OK;
}

i2c_master_bus_handle_t i2c_bus_get_handle(void)
{
    return s_bus_handle;
}
