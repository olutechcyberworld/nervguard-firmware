#include "vibration_driver.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "vibration_driver";

// GPIO11, locked in circuit_design_handoff.md. Do not change without a
// formal circuit revision. HIGH = motor on (via the NPN transistor
// driver — see the circuit doc for D1, the flyback diode, which is why
// this is safe to switch directly from a GPIO).
#define VIBRATION_GPIO 11

void vibration_set(bool on)
{
    gpio_set_level(VIBRATION_GPIO, on ? 1 : 0);
}

void vibration_pulse(uint32_t duration_ms)
{
    vibration_set(true);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    vibration_set(false);
}

esp_err_t vibration_driver_init(void)
{
    gpio_config_t config = {
        .pin_bit_mask = (1ULL << VIBRATION_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO%d: %s", VIBRATION_GPIO, esp_err_to_name(err));
        return err;
    }

    vibration_set(false);  // ensure it starts OFF, not whatever the pin defaulted to

    // First-ever real-hardware exercise of this subsystem — a short,
    // gentle confirmation pulse, per hardware_bringup_handoff.md's own
    // note that this circuit was accepted without independent testing.
    // If the motor doesn't buzz briefly right at boot, check D1 (the
    // flyback diode) and Q1 (the transistor) before anything else.
    ESP_LOGI(TAG, "Vibration motor ready on GPIO%d — sending a brief confirmation pulse", VIBRATION_GPIO);
    vibration_pulse(150);

    return ESP_OK;
}
