#include "ds18b20_driver.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "onewire_bus.h"
#include "ds18b20.h"

// GPIO5, locked in circuit_design_handoff.md, do not change without a
// formal circuit revision.
#define DS18B20_GPIO_PIN 5

static const char *TAG = "ds18b20_driver";

// --- Shared state between the background reading task and whoever asks
//     for the current temperature. Protected by a mutex because two
//     different pieces of code touch it: the background task writes to
//     it, and the feature-extraction code reads from it.
typedef struct {
    float celsius;
    int64_t timestamp_us;   // when this real reading was taken
    bool valid;             // false until the very first real reading lands
} temp_sample_t;

static SemaphoreHandle_t s_state_mutex = NULL;
static temp_sample_t s_prev_sample = {0};   // second-to-last real reading
static temp_sample_t s_curr_sample = {0};   // most recent real reading
static volatile temp_upsample_method_t s_upsample_method = TEMP_UPSAMPLE_HOLD_LAST;

static onewire_bus_handle_t s_bus = NULL;
static ds18b20_device_handle_t s_device = NULL;

// --- Background task: talks to the physical sensor.
//
// PLAIN-LANGUAGE SUMMARY OF THIS LOOP:
// Ask the sensor to take a reading (this takes 750ms in 12-bit mode
// because that is a hardware limit, not a setting we can change).
// Once it is done, read the number, store it as the new "most recent
// real reading," and remember the previous one too, since the
// gap-filling logic needs both to guess in-between values.
static void ds18b20_reading_task(void *arg)
{
    while (1) {
        int64_t loop_start_us = esp_timer_get_time();

        esp_err_t err = ds18b20_trigger_temperature_conversion(s_device);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to start a temperature reading: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // No separate wait here. ds18b20_trigger_temperature_conversion()
        // already blocks internally for the full ~750ms 12-bit conversion
        // time before returning — confirmed directly against real timing
        // data on this board. An earlier version of this driver added its
        // own additional 750ms wait on top of that, which doubled the real
        // gap between sensor readings to ~1.76s instead of the intended
        // ~1s. See hardware_bringup_handoff.md-equivalent note: always
        // measure, don't assume.

        float reading_celsius = 0.0f;
        err = ds18b20_get_temperature(s_device, &reading_celsius);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read temperature value: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        int64_t now_us = esp_timer_get_time();

        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
        s_prev_sample = s_curr_sample;
        s_curr_sample.celsius = reading_celsius;
        s_curr_sample.timestamp_us = now_us;
        s_curr_sample.valid = true;
        xSemaphoreGive(s_state_mutex);

        // Small pause before starting the next conversion. Total loop time
        // is now roughly 799ms (trigger call, conversion wait included) +
        // ~12ms (read) + this 200ms pause, landing close to the intended
        // "about once a second" real sampling rate.
        vTaskDelay(pdMS_TO_TICKS(200));

        int64_t loop_end_us = esp_timer_get_time();
        ESP_LOGI(TAG, "[ds18b20] real reading: %.4f C (loop took %lld ms)",
                 reading_celsius, (loop_end_us - loop_start_us) / 1000);
    }
}

esp_err_t ds18b20_driver_init(void)
{
    s_state_mutex = xSemaphoreCreateMutex();
    if (s_state_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    onewire_bus_config_t bus_config = {
        .bus_gpio_num = DS18B20_GPIO_PIN,
    };
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10,
    };
    esp_err_t err = onewire_new_bus_rmt(&bus_config, &rmt_config, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not set up the 1-Wire bus: %s", esp_err_to_name(err));
        return err;
    }

    // Single device on the bus, per circuit_design_handoff.md, so we walk
    // the 1-Wire device list looking for the first one that identifies
    // itself as a DS18B20, per the pattern documented by the component
    // itself (espressif/ds18b20, v0.4.0 readme).
    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_device = {0};
    err = onewire_new_device_iter(s_bus, &iter);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start searching the 1-Wire bus: %s", esp_err_to_name(err));
        return err;
    }

    esp_err_t search_result = ESP_OK;
    bool found = false;
    do {
        search_result = onewire_device_iter_get_next(iter, &next_device);
        if (search_result == ESP_OK) {
            ds18b20_config_t ds_config = {0};
            if (ds18b20_new_device_from_enumeration(&next_device, &ds_config, &s_device) == ESP_OK) {
                found = true;
                break;
            }
            // Device responded but isn't a DS18B20 — keep searching.
        }
    } while (search_result != ESP_ERR_NOT_FOUND);
    onewire_del_device_iter(iter);

    if (!found) {
        ESP_LOGE(TAG, "No DS18B20 found on GPIO%d. Check R4 pull-up and wiring.", DS18B20_GPIO_PIN);
        return ESP_ERR_NOT_FOUND;
    }

    // NOTE: this component's published usage example does not show an
    // explicit resolution-setting call, and 12-bit is the DS18B20's own
    // documented power-on default, which is what this project needs
    // anyway. TODO: once this component has downloaded locally, grep its
    // header (find it under the PlatformIO/ESP-IDF managed_components
    // folder) for any resolution-related function, in case one exists
    // and should be called explicitly for certainty rather than relying
    // on the power-on default.

    BaseType_t ok = xTaskCreate(ds18b20_reading_task, "ds18b20_read", 3072, NULL, 5, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Could not start the background reading task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Temperature sensor ready on GPIO%d, 12-bit resolution", DS18B20_GPIO_PIN);
    return ESP_OK;
}

void ds18b20_set_upsample_method(temp_upsample_method_t method)
{
    s_upsample_method = method;
    ESP_LOGI(TAG, "Gap-filling method set to: %s",
             method == TEMP_UPSAMPLE_HOLD_LAST ? "HOLD_LAST" : "LINEAR");
}

temp_upsample_method_t ds18b20_get_upsample_method(void)
{
    return s_upsample_method;
}

bool ds18b20_get_last_real_reading(float *out_celsius, uint32_t *out_age_ms)
{
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    bool valid = s_curr_sample.valid;
    float celsius = s_curr_sample.celsius;
    int64_t ts_us = s_curr_sample.timestamp_us;
    xSemaphoreGive(s_state_mutex);

    if (!valid) {
        return false;
    }

    if (out_celsius) {
        *out_celsius = celsius;
    }
    if (out_age_ms) {
        int64_t age_us = esp_timer_get_time() - ts_us;
        *out_age_ms = (uint32_t)(age_us / 1000);
    }
    return true;
}

bool ds18b20_get_temp_4hz(float *out_celsius)
{
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    temp_sample_t prev = s_prev_sample;
    temp_sample_t curr = s_curr_sample;
    temp_upsample_method_t method = s_upsample_method;
    xSemaphoreGive(s_state_mutex);

    if (!curr.valid) {
        // No real reading has landed yet. This only happens for roughly
        // the first second after startup.
        return false;
    }

    if (method == TEMP_UPSAMPLE_HOLD_LAST || !prev.valid) {
        // Method 1: just repeat the last real number. Also the fallback
        // if we don't have two real readings yet to guess between.
        *out_celsius = curr.celsius;
        return true;
    }

    // Method 2: project the temperature forward using the recent rate
    // of change.
    //
    // BUG FIX (found via real-hardware A/B testing): the original version
    // of this code tried to "fill in the gap" BETWEEN the previous real
    // reading and the current one. But by the time this function is ever
    // called, "right now" has already moved past both of those points in
    // time — there is no gap left between them to fill in. The old code
    // clamped its guess back down to the current reading almost
    // immediately, which meant it was silently behaving exactly like
    // "repeat the last number" the whole time, even though it looked
    // like it was doing something different.
    //
    // The correct approach: measure how fast temperature was changing
    // between the last two real readings (degrees per second), then
    // project that same rate forward from the most recent real reading
    // up to right now. This genuinely guesses ahead instead of only
    // ever looking backward.
    int64_t now_us = esp_timer_get_time();
    int64_t gap_us = curr.timestamp_us - prev.timestamp_us;
    if (gap_us <= 0) {
        // Guard against a bad timestamp gap; fall back to the safe method.
        *out_celsius = curr.celsius;
        return true;
    }

    float rate_per_us = (curr.celsius - prev.celsius) / (float)gap_us;
    int64_t elapsed_since_curr_us = now_us - curr.timestamp_us;
    if (elapsed_since_curr_us < 0) {
        elapsed_since_curr_us = 0;
    }

    *out_celsius = curr.celsius + rate_per_us * (float)elapsed_since_curr_us;
    return true;
}

bool ds18b20_get_temp_4hz_both(float *out_hold_last, float *out_linear)
{
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    temp_sample_t prev = s_prev_sample;
    temp_sample_t curr = s_curr_sample;
    xSemaphoreGive(s_state_mutex);

    if (!curr.valid) {
        return false;
    }

    // Method 1: repeat the last real number. Same for both outputs if we
    // don't have two real readings yet to guess between.
    *out_hold_last = curr.celsius;

    if (!prev.valid) {
        *out_linear = curr.celsius;
        return true;
    }

    // Method 2: project forward using the recent rate of change — same
    // corrected logic as ds18b20_get_temp_4hz() above.
    int64_t now_us = esp_timer_get_time();
    int64_t gap_us = curr.timestamp_us - prev.timestamp_us;
    if (gap_us <= 0) {
        *out_linear = curr.celsius;
        return true;
    }

    float rate_per_us = (curr.celsius - prev.celsius) / (float)gap_us;
    int64_t elapsed_since_curr_us = now_us - curr.timestamp_us;
    if (elapsed_since_curr_us < 0) {
        elapsed_since_curr_us = 0;
    }

    *out_linear = curr.celsius + rate_per_us * (float)elapsed_since_curr_us;
    return true;
}