#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
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
 *
 * BOOT-SEQUENCING FIX (this revision): the previous version of this
 * file initialised BLE last, after every sensor driver, and hard-halted
 * app_main() on the FIRST subsystem that failed to start — including
 * subsystems that have nothing to do with BLE. In practice this meant a
 * single sensor fault (most recently, DS18B20 failing to enumerate on
 * GPIO5) silently prevented the device from ever advertising at all,
 * which looked from the phone app like a BLE discoverability bug when
 * the BLE code itself was never actually reached, let alone at fault.
 *
 * This version inverts that: ble_gatt_server_init() now runs first,
 * immediately after NVS, and is the ONLY subsystem whose failure still
 * halts app_main() outright — because without it there is no way for
 * the app, or anyone without a serial cable, to observe any subsequent
 * failure. Every sensor and actuator driver after that point is
 * initialised independently; a failure in any one of them is logged,
 * reported to the app as an ERROR device state where the locked
 * contract defines a matching code, and then app_main() continues past
 * it rather than returning. Every per-tick use of a driver later in
 * this file is now guarded by that driver's own "_ok" flag, since a
 * driver that failed init must never be called into during the main
 * loop.
 */

// Per-subsystem init success, checked before every per-tick use of that
// subsystem in the main loop below. false means "skip this subsystem
// entirely, but keep running everything else."
static bool s_ds18b20_ok = false;
static bool s_max30102_ok = false;
static bool s_mpu6500_ok = false;
static bool s_eda_ok = false;
static bool s_motor_ok = false;
static bool s_led_ok = false;
static bool s_feature_extraction_ok = false;
static bool s_tflite_ok = false;

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

    // BLE now comes up FIRST, immediately after NVS, and before any
    // sensor. This is deliberately still fatal on failure: if BLE
    // itself cannot start, there is no remaining channel (short of a
    // serial cable) through which any later failure could be observed,
    // so continuing silently would be strictly worse than halting here.
    esp_err_t err = ble_gatt_server_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BLE GATT server failed to start. Halting here so the "
                       "failure is obvious rather than silently continuing.");
        return;
    }

    // From this point on, every subsystem failure is non-fatal to
    // app_main() itself. Each is logged, reported to the app as an
    // ERROR device state where the locked contract
    // (ble_gatt_contract_handoff.md) defines a matching error code, and
    // then execution continues to the next subsystem.

    err = ds18b20_driver_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Temperature sensor failed to start. Continuing without it "
                       "— BLE stays up and reports ERROR 0x02 (1-Wire failure).");
        ble_gatt_server_set_error(0x02);  // ERR_ONE_WIRE_FAILURE, per the locked contract
    } else {
        s_ds18b20_ok = true;
        ds18b20_set_upsample_method(TEMP_UPSAMPLE_HOLD_LAST);  // locked, see ds18b20_driver.h
    }

    err = max30102_driver_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Heart rate sensor failed to start. Continuing without it "
                       "— BLE stays up and reports ERROR 0x01 (I2C bus failure).");
        ble_gatt_server_set_error(0x01);  // ERR_I2C_BUS_FAILURE, per the locked contract
    } else {
        s_max30102_ok = true;
    }

    err = mpu6500_driver_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Movement sensor failed to start. Continuing without it "
                       "— BLE stays up and reports ERROR 0x01 (I2C bus failure, "
                       "shared bus with the heart rate sensor).");
        ble_gatt_server_set_error(0x01);  // ERR_I2C_BUS_FAILURE, per the locked contract
    } else {
        s_mpu6500_ok = true;
    }

    err = eda_driver_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "EDA sensor failed to start. Continuing without it "
                       "— BLE stays up and reports ERROR 0x03 (EDA ADC failure).");
        ble_gatt_server_set_error(0x03);  // ERR_EDA_ADC_FAILURE, per the locked contract
    } else {
        s_eda_ok = true;
    }

    // NOTE — open item, not silently resolved: ble_gatt_contract_handoff.md's
    // Error Code Reference table has no code covering a vibration motor
    // or RGB LED init failure. Both are simple, low-risk digital GPIO
    // outputs with no shared bus, so a failure here is logged only and
    // does NOT push the device into the ERROR state — inventing an
    // unlocked error code (e.g. 0x07/0x08) is a contract change and
    // should be a deliberate decision, not something this file decides
    // unilaterally. Neither driver's failure blocks anything downstream
    // of it either way.
    err = vibration_driver_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Vibration motor failed to start. Continuing without it. "
                       "No locked contract error code exists for this — see the "
                       "note above this block before adding one.");
    } else {
        s_motor_ok = true;
    }

    err = rgb_led_driver_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RGB LED failed to start. Continuing without it. "
                       "No locked contract error code exists for this — see the "
                       "note above this block before adding one.");
    } else {
        s_led_ok = true;
    }

    err = feature_extraction_init();
    if (err != ESP_OK) {
        // Also currently uncovered by a locked contract error code,
        // same caveat as the motor/LED case above — but unlike those
        // two, this failure is NOT low-risk: without feature extraction
        // there can be no inference, so this is effectively the whole
        // monitoring pipeline being unavailable. Worth prioritising a
        // contract revision for this one specifically.
        ESP_LOGE(TAG, "Feature extraction failed to start. Continuing without it, "
                       "but this disables the entire monitoring pipeline for this "
                       "boot. No locked contract error code exists for this yet.");
    } else {
        s_feature_extraction_ok = true;
    }

    err = tflite_inference_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TFLite inference failed to start. Continuing without it "
                       "— BLE stays up and reports ERROR 0x04 (TFLite allocation "
                       "failure), which the contract already documents as "
                       "unrecoverable for this boot.");
        ble_gatt_server_set_error(0x04);  // ERR_TFLITE_ALLOC_FAILURE, per the locked contract
    } else {
        s_tflite_ok = true;
    }

    ESP_LOGI(TAG, "Boot summary — DS18B20:%s MAX30102:%s MPU6500:%s EDA:%s "
                   "Motor:%s LED:%s FeatureExtraction:%s TFLite:%s. BLE is up "
                   "regardless of the above and advertising as \"NervGuard\".",
              s_ds18b20_ok ? "OK" : "FAILED", s_max30102_ok ? "OK" : "FAILED",
              s_mpu6500_ok ? "OK" : "FAILED", s_eda_ok ? "OK" : "FAILED",
              s_motor_ok ? "OK" : "FAILED", s_led_ok ? "OK" : "FAILED",
              s_feature_extraction_ok ? "OK" : "FAILED", s_tflite_ok ? "OK" : "FAILED");

    // ir_samples, accel_samples, and eda_samples used to live here as
    // static buffers for this function's own direct sensor reads.
    // REMOVED as part of this revision's fix: main.c no longer reads
    // any of the three multi-sample sensors directly. feature_extraction.c
    // is now the sole reader of each, and this function gets everything
    // it needs for its console lines from the three summary structs
    // populated there each tick. See feature_extraction.c's doc comment
    // above s_last_hr_summary for the full history of why this changed.
    static feature_vector_t latest_features;
    static float feature_array[13];

    uint8_t window_sequence = 0;

    // --- Periodic results-table checkpoint (added for stress-induction
    // test logging) ---
    //
    // WHY THIS EXISTS: the per-tick sensor lines and per-window feature
    // lines are useful for debugging but far too dense to manually
    // extract a results table from, especially for a multi-minute test
    // where the terminal's scrollback may not even retain the whole
    // session. This adds ONE compact, consistently-formatted line every
    // 5 minutes of device uptime, tagged [RESULT-LOG] so it can be
    // grepped out of a full log even if everything else got mixed
    // together or partially lost. Each line is self-contained (carries
    // its own elapsed-time stamp), so losing earlier scrollback does not
    // make a later line unusable — unlike the per-tick lines, which only
    // make sense with surrounding context.
    //
    // stress_class and probability are captured here into STATIC
    // "last known" variables specifically because, in the loop body
    // below, they are otherwise only ever local to the single iteration
    // that produced a fresh window — they do not normally persist until
    // the next 5-minute checkpoint fires, which will usually land
    // between windows (windows arrive every 5s; checkpoints every 5min).
    static int s_last_stress_class = 0;
    static float s_last_probability = 0.0f;
    static int64_t s_last_checkpoint_us = 0;
    #define RESULT_LOG_INTERVAL_US (5LL * 60LL * 1000000LL)  // 5 minutes

    while (1) {
        float temp_c = 0.0f;
        if (s_ds18b20_ok) {
            if (ds18b20_get_temp_4hz(&temp_c)) {
                ESP_LOGI(TAG, "Temp: %.4f C", temp_c);
            } else {
                ESP_LOGI(TAG, "Temp: waiting for first real reading...");
            }
        }

        // --- Full pipeline: feature extraction -> inference -> BLE ---
        //
        // feature_extraction_tick() must run every single loop (250ms),
        // since it's what feeds fresh sensor data into the 60-second
        // rolling window. It only actually PRODUCES a new 13-number
        // feature vector once every 5 seconds (20 ticks) — most calls
        // do nothing more than quietly top up the window. Guarded on
        // s_feature_extraction_ok: if feature_extraction_init() failed
        // at boot, ticking it further is undefined and must be skipped
        // entirely for the rest of this boot.
        //
        // ORDERING FIX: this must run BEFORE the three console-summary
        // blocks below can show THIS tick's data. feature_extraction_tick()
        // is what actually performs the heart-rate/movement/EDA reads
        // and populates the summaries those blocks display; it used to
        // run AFTER this main.c file's own direct sensor reads for those
        // same three console lines, which is what caused this session's
        // starvation bug — see feature_extraction.c's doc comment above
        // s_last_hr_summary for the full history.
        if (s_feature_extraction_ok) {
            feature_extraction_tick();
        }

        // Heart-rate console line now comes from feature_extraction's
        // own summary of the ONE read it performs each tick, rather
        // than a second, separate call to max30102_read_ir_samples()
        // from here. FIX: both calls used to draw from the same
        // consuming ring buffer, and whichever ran first — this one,
        // since it used to run before feature_extraction_tick() below
        // — took most of the tick's samples, leaving
        // feed_heart_rate() starved (confirmed directly via
        // [HRV-DIAG-BATCH] logging: ~2 samples reaching it on ticks
        // this line reported 24-30 for). See feature_extraction.c's
        // doc comment above s_last_hr_summary for the full history.
        // This line now only reads a summary struct, not the sensor.
        hr_batch_summary_t hr_summary = {0};
        bool have_hr_batch = feature_extraction_get_last_hr_batch(&hr_summary);
        if (have_hr_batch) {
            ESP_LOGI(TAG, "Heart rate: %u samples, latest = %lu, range in this batch = %lu (min %lu, max %lu)",
                      (unsigned)hr_summary.count, (unsigned long)hr_summary.latest,
                      (unsigned long)(hr_summary.max - hr_summary.min),
                      (unsigned long)hr_summary.min, (unsigned long)hr_summary.max);
        } else if (s_max30102_ok) {
            ESP_LOGI(TAG, "Heart rate: no new samples yet...");
        }

        // Movement console line, same fix, same reasoning: this used to
        // call mpu6500_read_samples() directly, a second, separate read
        // of the same consuming ring buffer feed_accel() also reads
        // from inside feature_extraction_tick(). Now reads the summary
        // feature_extraction.c already computed from its own single
        // authoritative read.
        accel_batch_summary_t accel_summary = {0};
        bool have_accel_batch = feature_extraction_get_last_accel_batch(&accel_summary);
        if (have_accel_batch) {
            ESP_LOGI(TAG, "Movement: %u samples, avg magnitude = %.4f g (min %.4f, max %.4f)",
                      (unsigned)accel_summary.count, accel_summary.avg_magnitude_g,
                      accel_summary.min_magnitude_g, accel_summary.max_magnitude_g);
        } else if (s_mpu6500_ok) {
            ESP_LOGI(TAG, "Movement: no new samples yet...");
        }

        // EDA console line, same fix, same reasoning: this used to call
        // eda_read_samples() directly, a second, separate read of the
        // same consuming ring buffer feed_eda() also reads from inside
        // feature_extraction_tick(). Now reads the summary
        // feature_extraction.c already computed from its own single
        // authoritative read.
        eda_batch_summary_t eda_summary = {0};
        bool have_eda_batch = feature_extraction_get_last_eda_batch(&eda_summary);
        if (have_eda_batch) {
            ESP_LOGI(TAG, "EDA: %u samples, latest = %.4fV -> %.0f ohms -> %.3f uS%s%s, "
                           "%u/%u outside calibrated range, %u/%u rejected as noise",
                      (unsigned)eda_summary.count, eda_summary.latest_voltage,
                      eda_summary.latest_resistance_ohms, eda_summary.latest_conductance_us,
                      eda_summary.latest_calibrated ? "" : " (EXTRAPOLATED)",
                      eda_summary.latest_artifact_rejected ? " (HELD - noise rejected)" : "",
                      (unsigned)eda_summary.uncalibrated_count, (unsigned)eda_summary.count,
                      (unsigned)eda_summary.rejected_count, (unsigned)eda_summary.count);
        } else if (s_eda_ok) {
            ESP_LOGI(TAG, "EDA: no new samples yet...");
        }

        // Wear detection needs the current temperature every tick.
        // Guarded on s_ds18b20_ok: without a working sensor, temp_c
        // above is never populated (stays 0.0f), and feeding that in
        // would read as "below the NOT_WORN threshold" and could
        // spuriously drive a wear-state transition despite carrying no
        // real information about the device being on the wrist.
        if (s_ds18b20_ok) {
            ble_gatt_server_update_wear_detection(temp_c);
        }

        if (s_feature_extraction_ok && feature_extraction_get_latest(&latest_features)) {
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
            if (s_tflite_ok && tflite_inference_run(feature_array, &stress_class, &probability)) {
                ESP_LOGI(TAG, "Inference: class=%d probability=%.4f", stress_class, probability);
                ble_gatt_server_notify_inference(stress_class, probability, window_sequence);

                // Feeds the periodic [RESULT-LOG] checkpoint below —
                // see the doc comment above s_last_stress_class's
                // declaration for why this capture is needed here.
                s_last_stress_class = stress_class;
                s_last_probability = probability;

                // Simple visual status indicator only — this is
                // separate from, and does not drive, the app's own
                // intervention logic (mild/intensive pools, cooldowns,
                // escalation), which stays entirely app-side per
                // chat2_firmware_handoff.md's locked design. The motor
                // is deliberately NOT auto-triggered here for the same
                // reason — see vibration_driver.h. Guarded on
                // s_led_ok since rgb_led_driver_init() may not have
                // succeeded this boot.
                if (s_led_ok) {
                    switch (stress_class) {
                        case 0: rgb_led_set(LED_GREEN);  break;
                        case 1: rgb_led_set(LED_YELLOW); break;
                        case 2: rgb_led_set(LED_RED);    break;
                        default: break;
                    }
                }
            } else if (s_tflite_ok) {
                ESP_LOGW(TAG, "Inference failed for this window");
            }
        }

        // --- Periodic results-table checkpoint ---
        // See the doc comment above s_last_stress_class's declaration.
        // Fires once every 5 minutes of device uptime, independent of
        // the 5-second window/inference cadence above. One self-
        // contained, grep-able line: [RESULT-LOG] elapsed=HH:MM:SS
        // followed by the most recent value of every field a results
        // table needs. Elapsed time is derived from esp_timer_get_time()
        // (microseconds since boot), not a loop-iteration count, so it
        // stays accurate to real wall-clock time regardless of any
        // single iteration running slightly longer than 250ms.
        int64_t now_us = esp_timer_get_time();
        if (now_us - s_last_checkpoint_us >= RESULT_LOG_INTERVAL_US) {
            s_last_checkpoint_us = now_us;
            int64_t elapsed_s = now_us / 1000000LL;
            int hh = (int)(elapsed_s / 3600);
            int mm = (int)((elapsed_s % 3600) / 60);
            int ss = (int)(elapsed_s % 60);
            ESP_LOGI(TAG, "[RESULT-LOG] elapsed=%02d:%02d:%02d seq=%u class=%d prob=%.4f "
                           "eda_uS=%.3f hrv_rmssd_ms=%.1f hrv_mean_rr_ms=%.1f hrv_sdnn_ms=%.1f hrv_sd2_ms=%.1f "
                           "temp_C=%.3f acc_mag_mean_g=%.3f acc_mag_std_g=%.3f acc_activity=%.0f",
                      hh, mm, ss, window_sequence, s_last_stress_class, s_last_probability,
                      latest_features.eda_mean,
                      latest_features.hrv_rmssd, latest_features.hrv_mean_rr,
                      latest_features.hrv_sdnn, latest_features.hrv_sd2,
                      latest_features.temp_mean,
                      latest_features.acc_mag_mean, latest_features.acc_mag_std,
                      latest_features.acc_activity);
        }

        // 250ms = four times a second. This is the cadence feature
        // extraction is built around — do not change this delay without
        // also revisiting feature_extraction.c's tick-based assumptions.
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}