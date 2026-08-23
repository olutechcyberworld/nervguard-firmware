#include "rgb_led_driver.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "rgb_led_driver";

// GPIO12/13/14, locked in circuit_design_handoff.md. Do not change
// without a formal circuit revision. Common cathode with current-
// limiting resistors already on the board — HIGH lights each channel.
#define LED_RED_GPIO   12
#define LED_GREEN_GPIO 13
#define LED_BLUE_GPIO  14

void rgb_led_set(led_color_t color)
{
    bool r = false, g = false, b = false;
    switch (color) {
        case LED_RED:    r = true; break;
        case LED_GREEN:  g = true; break;
        case LED_BLUE:   b = true; break;
        case LED_WHITE:  r = g = b = true; break;
        case LED_YELLOW: r = g = true; break;
        case LED_PURPLE: r = b = true; break;
        case LED_CYAN:   g = b = true; break;
        case LED_OFF:
        default:
            break;
    }
    gpio_set_level(LED_RED_GPIO, r ? 1 : 0);
    gpio_set_level(LED_GREEN_GPIO, g ? 1 : 0);
    gpio_set_level(LED_BLUE_GPIO, b ? 1 : 0);
}

esp_err_t rgb_led_driver_init(void)
{
    gpio_config_t config = {
        .pin_bit_mask = (1ULL << LED_RED_GPIO) | (1ULL << LED_GREEN_GPIO) | (1ULL << LED_BLUE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LED GPIOs: %s", esp_err_to_name(err));
        return err;
    }

    // First-ever real-hardware exercise of this subsystem: cycle
    // through each channel individually, briefly, so it's obvious at a
    // glance whether all three colours are wired and working, rather
    // than only ever seeing combined colours later and not knowing
    // which single channel might be at fault if something looks off.
    ESP_LOGI(TAG, "RGB LED ready on GPIO%d/%d/%d — cycling R/G/B to confirm all three channels",
              LED_RED_GPIO, LED_GREEN_GPIO, LED_BLUE_GPIO);
    rgb_led_set(LED_RED);   vTaskDelay(pdMS_TO_TICKS(200));
    rgb_led_set(LED_GREEN); vTaskDelay(pdMS_TO_TICKS(200));
    rgb_led_set(LED_BLUE);  vTaskDelay(pdMS_TO_TICKS(200));
    rgb_led_set(LED_OFF);

    return ESP_OK;
}
