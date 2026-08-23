#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "ds18b20_driver.h"
#include "max30102_driver.h"
#include "mpu6500_driver.h"
#include "eda_driver.h"
#include "vibration_driver.h"
#include "rgb_led_driver.h"
#include "feature_extraction.h"
#include "tflite_inference.h"
#include "ble_gatt_server.h"

static const char *TAG = "main";

/*
 * PLAIN-LANGUAGE SUMMARY OF THIS FILE:
 * This is the very first thing that runs when the ESP32-S3 boots, and
 * it's now the FULL pipeline, start to finish:
 *
 *   Four sensors (temperature, heart rate, movement, skin conductance)
 *   -> feature_extraction (turns 60 seconds of raw data into 13 numbers,
 *      once every 5 seconds)
 *   -> tflite_inference (runs those 13 numbers through the trained
 *      model, gets back a stress class and a probability)
 *   -> ble_gatt_server (pushes the result out to the phone app, and
 *      handles everything the app can ask the device to do)
 *
 * Every sensor driver was already confirmed working on its own, on real
 * hardware, before reaching this point — see each driver's own header
 * file for that history. This is the first time the whole chain runs
 * together, end to end.
 */
void app_main(void)
{
    ESP_LOGI(TAG, "NervGuard firmware starting...");

    // BUG FIX (found via real-hardware testing): this chip's small
    // persistent-storage area (used for calibration data, and by the
    // Bluetooth stack internally too) needs one project-wide "get
    // ready" step before ANY component is allowed to open it.
    // ble_gatt_server_init() tried to open its own storage namespace
    // without this ever having run, and correctly refused with
    // ESP_ERR_NVS_NOT_INITIALIZED rather than silently failing in a
    // more confusing way later.
    //
    // The two special-case results below are the standard, expected
    // recovery path: they mean the storage area was left in a state
    // this exact chip's flash layout doesn't recognise (e.g. after a
    // partition table change), so it's erased and reinitialised fresh
    // — this is safe and does not affect anything else on the chip.
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition needs a fresh start — erasing and reinitialising");
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    if (nvs_err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s. Halting here so the failure is obvious "
                       "rather than silently continuing.", esp_err_to_name(nvs_err));
        return;
    }

    esp_err_t err = ds18b20_driver_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Temperature sensor failed to start. Halting here so the "
                       "failure is obvious rather than silently continuing.");
        return;
    }
    ds18b20_set_upsample_method(TEMP_UPSAMPLE_HOLD_LAST);  // locked, see ds18b20_driver.h

    err = max30102_driver_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Heart rate sensor failed to start. Halting here so the "
                       "failure is obvious rather than silently continuing.");
        return;
    }

    err = mpu6500_driver_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Movement sensor failed to start. Halting here so the "
                       "failure is obvious rather than silently continuing.");
        return;
    }

    err = eda_driver_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "EDA sensor failed to start. Halting here so the "
                       "failure is obvious rather than silently continuing.");
        return;
    }

    err = vibration_driver_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Vibration motor failed to start. Halting here so the "
                       "failure is obvious rather than silently continuing.");
        return;
    }

    err = rgb_led_driver_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RGB LED failed to start. Halting here so the "
                       "failure is obvious rather than silently continuing.");
        return;
    }

    err = feature_extraction_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Feature extraction failed to start. Halting here so the "
                       "failure is obvious rather than silently continuing.");
        return;
    }

    err = tflite_inference_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TFLite inference failed to start. Halting here so the "
                       "failure is obvious rather than silently continuing.");
        return;
    }

    err = ble_gatt_server_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BLE GATT server failed to start. Halting here so the "
                       "failure is obvious rather than silently continuing.");
        return;
    }

    // These were originally plain local variables here. That put over
    // 2KB of buffer space directly on app_main's own small task stack,
    // which was fine with two sensors' worth of buffers but overflowed
    // that stack once the third (larger) movement buffer was added,
    // causing the reboot loop seen on real hardware. Marking them
    // static moves them into a separate, much larger area of memory
    // instead, leaving app_main's own stack untouched.
    static uint32_t ir_samples[100];
    static mpu6500_sample_t accel_samples[150];
    static eda_sample_t eda_samples[150];
    static feature_vector_t latest_features;
    static float feature_array[13];

    uint8_t window_sequence = 0;

    while (1) {
        float temp_c = 0.0f;
        if (ds18b20_get_temp_4hz(&temp_c)) {
            ESP_LOGI(TAG, "Temp: %.4f C", temp_c);
        } else {
            ESP_LOGI(TAG, "Temp: waiting for first real reading...");
        }

        // Drain whatever heart-rate samples have piled up since the last
        // time we checked (the sensor fills these at a steady 100/sec on
        // its own, in hardware, independent of this loop's own timing).
        size_t ir_count = 0;
        max30102_read_ir_samples(ir_samples, 100, &ir_count);
        if (ir_count > 0) {
            uint32_t min_val = ir_samples[0];
            uint32_t max_val = ir_samples[0];
            for (size_t i = 1; i < ir_count; i++) {
                if (ir_samples[i] < min_val) min_val = ir_samples[i];
                if (ir_samples[i] > max_val) max_val = ir_samples[i];
            }
            ESP_LOGI(TAG, "Heart rate: %u samples, latest = %lu, range in this batch = %lu (min %lu, max %lu)",
                      (unsigned)ir_count, (unsigned long)ir_samples[ir_count - 1],
                      (unsigned long)(max_val - min_val), (unsigned long)min_val, (unsigned long)max_val);
        } else {
            ESP_LOGI(TAG, "Heart rate: no new samples yet...");
        }

        // Same drain-and-summarise approach for movement. A stationary
        // device should show a magnitude very close to 1.0g (gravity)
        // with almost no variation. Picking the device up and moving it
        // should push both the average and the swing in magnitude well
        // away from that quiet 1.0g baseline.
        size_t accel_count = 0;
        mpu6500_read_samples(accel_samples, 150, &accel_count);
        if (accel_count > 0) {
            float mag_sum = 0.0f, mag_min = 0.0f, mag_max = 0.0f;
            for (size_t i = 0; i < accel_count; i++) {
                mpu6500_sample_t s = accel_samples[i];
                float mag = sqrtf(s.x_g * s.x_g + s.y_g * s.y_g + s.z_g * s.z_g);
                if (i == 0) { mag_min = mag; mag_max = mag; }
                if (mag < mag_min) mag_min = mag;
                if (mag > mag_max) mag_max = mag;
                mag_sum += mag;
            }
            float mag_avg = mag_sum / (float)accel_count;
            ESP_LOGI(TAG, "Movement: %u samples, avg magnitude = %.4f g (min %.4f, max %.4f)",
                      (unsigned)accel_count, mag_avg, mag_min, mag_max);
        } else {
            ESP_LOGI(TAG, "Movement: no new samples yet...");
        }

        // Same drain-and-summarise approach for skin conductance. Two
        // counts matter here: "outside calibrated range" tells us how
        // often real readings are landing beyond the three actually-
        // measured calibration points (see eda_driver.h) — a high count
        // is a sign the table genuinely needs those extra 50kΩ/220kΩ
        // points. "rejected as noise" tells us how often the filter is
        // catching an implausibly fast jump — see eda_driver.c for why
        // that filter exists and real hardware evidence for it.
        size_t eda_count = 0;
        eda_read_samples(eda_samples, 150, &eda_count);
        if (eda_count > 0) {
            eda_sample_t latest = eda_samples[eda_count - 1];
            size_t uncalibrated_count = 0;
            size_t rejected_count = 0;
            for (size_t i = 0; i < eda_count; i++) {
                if (!eda_samples[i].calibrated) uncalibrated_count++;
                if (eda_samples[i].artifact_rejected) rejected_count++;
            }
            ESP_LOGI(TAG, "EDA: %u samples, latest = %.4fV -> %.0f ohms -> %.3f uS%s%s, "
                           "%u/%u outside calibrated range, %u/%u rejected as noise",
                      (unsigned)eda_count, latest.voltage, latest.resistance_ohms, latest.conductance_us,
                      latest.calibrated ? "" : " (EXTRAPOLATED)",
                      latest.artifact_rejected ? " (HELD - noise rejected)" : "",
                      (unsigned)uncalibrated_count, (unsigned)eda_count,
                      (unsigned)rejected_count, (unsigned)eda_count);
        } else {
            ESP_LOGI(TAG, "EDA: no new samples yet...");
        }

        // --- Full pipeline: feature extraction -> inference -> BLE ---
        //
        // feature_extraction_tick() must run every single loop (250ms),
        // since it's what feeds fresh sensor data into the 60-second
        // rolling window. It only actually PRODUCES a new 13-number
        // feature vector once every 5 seconds (20 ticks) — most calls
        // do nothing more than quietly top up the window.
        feature_extraction_tick();

        // Wear detection needs the current temperature every tick,
        // regardless of whether a new feature window is ready.
        ble_gatt_server_update_wear_detection(temp_c);

        if (feature_extraction_get_latest(&latest_features)) {
            window_sequence++;

            // Feature order here MUST match feature_extraction.h's
            // locked field order exactly — this is what the model was
            // trained on.
            feature_array[0]  = latest_features.eda_mean;
            feature_array[1]  = latest_features.eda_std;
            feature_array[2]  = latest_features.eda_slope;
            feature_array[3]  = latest_features.eda_range;
            feature_array[4]  = latest_features.hrv_rmssd;
            feature_array[5]  = latest_features.hrv_mean_rr;
            feature_array[6]  = latest_features.hrv_sdnn;
            feature_array[7]  = latest_features.hrv_sd2;
            feature_array[8]  = latest_features.temp_mean;
            feature_array[9]  = latest_features.temp_slope;
            feature_array[10] = latest_features.acc_mag_mean;
            feature_array[11] = latest_features.acc_mag_std;
            feature_array[12] = latest_features.acc_activity;

            ESP_LOGI(TAG, "=== New 60s feature window (seq %u) === "
                           "EDA: mean=%.3f std=%.3f slope=%.5f range=%.3f | "
                           "HRV: rmssd=%.1f mean_rr=%.1f sdnn=%.1f sd2=%.1f | "
                           "TEMP: mean=%.3f slope=%.5f | "
                           "ACC: mag_mean=%.3f mag_std=%.3f activity=%.0f",
                      window_sequence,
                      latest_features.eda_mean, latest_features.eda_std,
                      latest_features.eda_slope, latest_features.eda_range,
                      latest_features.hrv_rmssd, latest_features.hrv_mean_rr,
                      latest_features.hrv_sdnn, latest_features.hrv_sd2,
                      latest_features.temp_mean, latest_features.temp_slope,
                      latest_features.acc_mag_mean, latest_features.acc_mag_std,
                      latest_features.acc_activity);

            // This also feeds the calibration accumulator internally
            // when the device is in the CALIBRATING state — see
            // ble_gatt_server.c.
            ble_gatt_server_notify_features(&latest_features, window_sequence);

            int stress_class = 0;
            float probability = 0.0f;
            if (tflite_inference_run(feature_array, &stress_class, &probability)) {
                ESP_LOGI(TAG, "Inference: class=%d probability=%.4f", stress_class, probability);
                ble_gatt_server_notify_inference(stress_class, probability, window_sequence);

                // Simple visual status indicator only — this is
                // separate from, and does not drive, the app's own
                // intervention logic (mild/intensive pools, cooldowns,
                // escalation), which stays entirely app-side per
                // chat2_firmware_handoff.md's locked design. The motor
                // is deliberately NOT auto-triggered here for the same
                // reason — see vibration_driver.h.
                switch (stress_class) {
                    case 0: rgb_led_set(LED_GREEN);  break;
                    case 1: rgb_led_set(LED_YELLOW); break;
                    case 2: rgb_led_set(LED_RED);    break;
                    default: break;
                }
            } else {
                ESP_LOGW(TAG, "Inference failed for this window");
            }
        }

        // 250ms = four times a second. This is the cadence feature
        // extraction is built around — do not change this delay without
        // also revisiting feature_extraction.c's tick-based assumptions.
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}