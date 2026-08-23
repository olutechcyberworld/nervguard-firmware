#pragma once

#include <stdbool.h>
#include "esp_err.h"

/*
 * DS18B20 skin temperature driver.
 *
 * PLAIN-LANGUAGE SUMMARY:
 * The physical sensor can only give us one real, trustworthy reading
 * per second (its 12-bit mode takes 750ms to complete a conversion).
 * The rest of the system needs a temperature value four times a second,
 * to match the other sensors.
 *
 * This driver handles that gap in two steps:
 *   1. A background task talks to the real sensor about once a second
 *      and stores the real reading.
 *   2. Whenever the feature-extraction code asks for "the temperature
 *      right now," this driver hands back a value using whichever
 *      gap-filling method is currently selected below.
 *
 * NOTHING IS LOCKED YET. Both gap-filling methods are implemented.
 * Which one we keep is decided after real-hardware testing, per the
 * open item carried forward from hardware bring-up.
 */

typedef enum {
    // LOCKED AS THE PRODUCTION METHOD (see hardware_bringup_handoff.md-
    // equivalent A/B test record). Repeat the last real measurement
    // until the next real one arrives. Never invents a number.
    //
    // Chosen over TEMP_UPSAMPLE_LINEAR after a real on-hardware
    // comparison during a genuine skin-contact temperature swing: the
    // linear method was found to systematically overshoot the true next
    // reading during the fastest phase of a real temperature change
    // (peak divergence +0.4988°C), which is precisely the phase most
    // relevant to detecting a real stress response via temp_slope.
    TEMP_UPSAMPLE_HOLD_LAST = 0,

    // NOT USED IN PRODUCTION. Kept in the driver in case a future
    // revision wants to revisit this with a curve-aware projection
    // instead of straight-line extrapolation. See ds18b20_get_temp_4hz_both()
    // for the side-by-side comparison tooling this decision was made with.
    TEMP_UPSAMPLE_LINEAR = 1,
} temp_upsample_method_t;

// Starts the background task that reads the real sensor roughly once
// a second. Call this once, during firmware startup.
esp_err_t ds18b20_driver_init(void);

// Switches which gap-filling method is active. Safe to call at any time,
// including while the system is running, so both methods can be
// A/B tested back to back on the real board without reflashing.
void ds18b20_set_upsample_method(temp_upsample_method_t method);

temp_upsample_method_t ds18b20_get_upsample_method(void);

// Returns the temperature to use "right now," in Celsius, using whichever
// gap-filling method is active. Call this once per 250ms feature-extraction
// tick (i.e. four times a second).
//
// Returns true and fills *out_celsius if a value is available.
// Returns false if the sensor has not produced its first real reading yet
// (this only happens briefly at startup).
bool ds18b20_get_temp_4hz(float *out_celsius);

// TEST-ONLY: computes BOTH gap-filling methods for the exact same instant,
// so they can be logged side by side and compared directly. Not meant for
// production use once a method is locked in; ds18b20_get_temp_4hz() above
// is the one real code should call going forward.
//
// Returns true and fills both outputs if a value is available.
// Returns false if the sensor has not produced its first real reading yet.
bool ds18b20_get_temp_4hz_both(float *out_hold_last, float *out_linear);

// Returns the single most recent REAL measurement (not gap-filled), and
// how many milliseconds ago it was taken. Useful for diagnostics and for
// the A/B comparison test itself.
bool ds18b20_get_last_real_reading(float *out_celsius, uint32_t *out_age_ms);