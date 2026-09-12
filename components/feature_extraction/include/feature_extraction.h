#pragma once

#include <stdbool.h>
#include <stddef.h>   // size_t, used by the batch summary structs below
#include <stdint.h>   // uint32_t, used by hr_batch_summary_t below
#include "esp_err.h"

/*
 * Feature extraction.
 *
 * PLAIN-LANGUAGE SUMMARY:
 * This is the piece that turns four raw sensor streams into the 13
 * numbers the stress-detection model actually looks at. Every 5
 * seconds, it looks back over the last 60 seconds of data and computes
 * one fresh set of 13 numbers — exact order and meaning locked in
 * chat2_firmware_handoff.md.
 *
 * HOW TO USE THIS:
 *   1. Call feature_extraction_init() once at startup.
 *   2. Call feature_extraction_tick() once every 250ms, from the same
 *      main loop that already drains the four sensor drivers. This is
 *      what feeds fresh samples into the windowing logic.
 *   3. Call feature_extraction_get_latest() whenever convenient (e.g.
 *      once per tick) to check whether a new 13-number feature vector
 *      became ready. It only returns true once per window — calling it
 *      again before the next one is ready just returns false.
 */

typedef struct {
    // Order matches the locked feature table in
    // chat2_firmware_handoff.md exactly — this is the order the model
    // itself expects. Do not reorder these fields.
    float eda_mean;
    float eda_std;
    float eda_slope;
    float eda_range;
    float hrv_rmssd;    // -1.0f if fewer than 30 clean heartbeats this window
    float hrv_mean_rr;  // -1.0f if fewer than 30 clean heartbeats this window
    float hrv_sdnn;     // -1.0f if fewer than 30 clean heartbeats this window
    float hrv_sd2;      // -1.0f if fewer than 30 clean heartbeats this window
    float temp_mean;
    float temp_slope;
    float acc_mag_mean;
    float acc_mag_std;
    float acc_activity;  // 1.0f if acc_mag_std > 0.02g, else 0.0f

    // Firmware-internal only — deliberately placed AFTER the locked 13
    // fields above and NOT included in feature_vector_to_array(), so it
    // never reaches the BLE wire format or the model input. Fraction of
    // this window's raw EDA samples that were extrapolated or noise-
    // rejected; consumed only by ble_gatt_server.c's calibration
    // validity check. See feature_extraction.c's feed_eda() for how
    // this is computed.
    float eda_reject_frac;
} feature_vector_t;

esp_err_t feature_extraction_init(void);

void feature_extraction_tick(void);

bool feature_extraction_get_latest(feature_vector_t *out);

// --- Per-sensor batch summaries, for console/diagnostic display only. ---
//
// ADDED this revision. feature_extraction.c is the SOLE caller of
// max30102_read_ir_samples(), eda_read_samples(), and
// mpu6500_read_samples() — all three underlying ring buffers are
// destructive on read (each call permanently removes whatever it
// returns), so exactly one reader per sensor is required. main.c
// previously called all three directly a second time, purely to print
// its own console line, which silently starved feature extraction of
// most of each tick's samples — confirmed directly via real-hardware
// [HRV-DIAG-BATCH] logging showing ~2 samples reaching feed_heart_rate()
// on ticks main.c's own line reported 24-30 samples for. These three
// structs are how main.c now gets what it needs for that same console
// output without ever touching a sensor driver a second time.
typedef struct {
    size_t count;
    uint32_t latest;
    uint32_t min;
    uint32_t max;
} hr_batch_summary_t;

typedef struct {
    size_t count;
    float avg_magnitude_g;
    float min_magnitude_g;
    float max_magnitude_g;
} accel_batch_summary_t;

typedef struct {
    size_t count;
    float latest_voltage;
    float latest_resistance_ohms;
    float latest_conductance_us;
    bool latest_calibrated;
    bool latest_artifact_rejected;
    size_t uncalibrated_count;
    size_t rejected_count;
} eda_batch_summary_t;

// Each returns true if this tick's read actually produced at least one
// sample. out is always fully written either way (count = 0 and all
// other fields zeroed on a false return), so a caller that skips the
// return-value check still gets a safe, well-defined struct.
bool feature_extraction_get_last_hr_batch(hr_batch_summary_t *out);
bool feature_extraction_get_last_accel_batch(accel_batch_summary_t *out);
bool feature_extraction_get_last_eda_batch(eda_batch_summary_t *out);