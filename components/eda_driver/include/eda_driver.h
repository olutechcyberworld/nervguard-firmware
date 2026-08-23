#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

/*
 * EDA (electrodermal activity / skin conductance) driver.
 *
 * PLAIN-LANGUAGE SUMMARY:
 * This sensor doesn't talk over a protocol like the others — it's just
 * a plain voltage on one pin (GPIO4), coming out of the LM358 amplifier
 * circuit built during hardware bring-up. This driver reads that
 * voltage, converts it into a skin resistance value using the real,
 * measured lookup table from that bring-up session (not a theoretical
 * formula — see hardware_bringup_handoff.md for why), and then into
 * "microsiemens" (µS), the standard unit for skin conductance used
 * throughout this project's ML pipeline.
 *
 * IMPORTANT — CALIBRATION TABLE IS STILL INCOMPLETE:
 * The lookup table below has only three real measured points. There is
 * a known, flagged gap between the 100kΩ and 470kΩ points — the widest
 * gap in the table — and the table doesn't yet reach down to 50kΩ.
 * Readings that fall in that gap, or beyond either end of the table,
 * are extrapolated rather than measured, and this driver marks them as
 * such (see eda_sample_t.calibrated below) rather than silently
 * presenting a guess as a confirmed value. Add the 50kΩ and 220kΩ
 * measured points to the table below as soon as they're available.
 * IMPORTANT — REAL ELECTRICAL NOISE, CONFIRMED ON HARDWARE:
 * Testing on the real board showed that once a finger actually bridges
 * the electrodes, the raw signal can swing wildly from one 10ms sample
 * to the next — far faster than real skin conductance can physically
 * change (which moves over whole seconds, not milliseconds). The most
 * likely cause is the body picking up ambient electrical noise from the
 * room through the open, unshielded breadboard wiring. This driver
 * filters that out: any single-step jump too large to be real is
 * treated as a noise artifact and replaced with the last trusted
 * reading, with every such replacement clearly flagged (see
 * eda_sample_t.artifact_rejected below) rather than silently smoothed
 * over.
 */

typedef struct {
    float voltage;          // Raw voltage read on GPIO4
    float resistance_ohms;  // Converted skin resistance
    float conductance_us;   // Converted skin conductance, in microsiemens
    bool calibrated;        // false if this reading fell outside the
                             // three actually-measured calibration
                             // points and had to be extrapolated
    bool artifact_rejected; // true if this reading changed too fast to
                             // be real skin conductance and was replaced
                             // with the last trusted value — see
                             // eda_driver.c for the reasoning and the
                             // real hardware evidence behind this filter
} eda_sample_t;

esp_err_t eda_driver_init(void);

// Pulls up to max_count fresh samples out of the internal buffer, in the
// order they were captured (oldest first). *out_count is set to however
// many were actually available (0 up to max_count).
esp_err_t eda_read_samples(eda_sample_t *out_buf, size_t max_count, size_t *out_count);

// Returns the single most recent sample, for quick diagnostics.
// Returns false if no sample has been captured yet.
bool eda_get_last_sample(eda_sample_t *out_sample);