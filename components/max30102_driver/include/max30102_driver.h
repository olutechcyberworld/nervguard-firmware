#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

/*
 * MAX30102 heart rate (PPG) driver.
 *
 * IMPORTANT CORRECTION FROM THE ORIGINAL CIRCUIT DESIGN DOCUMENT:
 * circuit_design_handoff.md says firmware should select a "Green LED
 * channel" on this sensor for the best perfusion signal. The real
 * MAX30102 does not have a green LED at all — confirmed directly
 * against the sensor's own documentation before writing this driver.
 * Only Red and Infrared exist on this chip. Green LED only exists on a
 * different, easily-confused sensor (the MAX30105).
 *
 * This driver uses the Infrared (IR) channel instead. This is the
 * standard substitute for heart-rate-only use on this exact sensor: IR
 * penetrates skin more deeply than Red, giving the better of the two
 * available perfusion signals.
 *
 * PLAIN-LANGUAGE SUMMARY OF HOW THIS DRIVER WORKS:
 * The sensor takes 100 readings every second on its own, entirely in
 * hardware, and stores them in its own small internal memory (a "FIFO").
 * This driver's background task periodically empties that memory into
 * its own buffer here in the ESP32-S3, so the rest of the firmware can
 * ask for however many fresh heart-rate samples it needs, whenever it's
 * ready for them, without ever talking to the sensor's registers
 * directly.
 */

esp_err_t max30102_driver_init(void);

// Pulls up to max_count fresh IR samples out of the internal buffer, in
// the order they were captured (oldest first). *out_count is set to
// however many were actually available (0 up to max_count).
esp_err_t max30102_read_ir_samples(uint32_t *out_buf, size_t max_count, size_t *out_count);

// Returns the single most recent IR sample, for quick diagnostics.
// Returns false if no sample has been captured yet.
bool max30102_get_last_ir_sample(uint32_t *out_value);
