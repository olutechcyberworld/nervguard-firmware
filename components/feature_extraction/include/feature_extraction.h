#pragma once

#include <stdbool.h>
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
} feature_vector_t;

esp_err_t feature_extraction_init(void);

void feature_extraction_tick(void);

bool feature_extraction_get_latest(feature_vector_t *out);
