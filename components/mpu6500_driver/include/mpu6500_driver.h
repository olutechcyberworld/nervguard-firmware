#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

/*
 * MPU6500 movement (accelerometer) driver.
 *
 * PLAIN-LANGUAGE SUMMARY:
 * This sensor measures how the wrist is moving along three directions
 * (X, Y, Z), in units of "g" (1.0g = the normal pull of gravity when
 * the device is sitting still). This driver reads it 100 times a
 * second, matching the acquisition rate the rest of the firmware
 * expects, and hands back already-converted, correct g values.
 *
 * IMPORTANT UNIT NOTE: ml_pipeline_documentation.md mentions dividing
 * raw counts by 64 to get g-units. That number is specific to the
 * Empatica E4 device used to record the original training data — a
 * different, unrelated sensor. This driver's numbers are already
 * correctly converted to g using the MPU6500's own real scale factor;
 * nothing downstream should divide by 64 again.
 *
 * Only the gyroscope's twin, the accelerometer, is used here — this
 * project has no use for rotation data, so the gyroscope is left
 * configured but unread.
 */

typedef struct {
    float x_g;
    float y_g;
    float z_g;
} mpu6500_sample_t;

esp_err_t mpu6500_driver_init(void);

// Pulls up to max_count fresh samples out of the internal buffer, in the
// order they were captured (oldest first). *out_count is set to however
// many were actually available (0 up to max_count).
esp_err_t mpu6500_read_samples(mpu6500_sample_t *out_buf, size_t max_count, size_t *out_count);

// Returns the single most recent sample, for quick diagnostics.
// Returns false if no sample has been captured yet.
bool mpu6500_get_last_sample(mpu6500_sample_t *out_sample);
