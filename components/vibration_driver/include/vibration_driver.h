#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/*
 * Vibration motor driver.
 *
 * PLAIN-LANGUAGE SUMMARY:
 * This is the simplest driver in the project: one GPIO pin (GPIO11)
 * driving a transistor that switches the motor on or off, per
 * circuit_design_handoff.md's Subsystem 5. HIGH = motor on,
 * LOW = motor off. There's no sensor to read here, nothing to
 * calibrate — just a switch.
 *
 * NOT YET EXERCISED ON REAL HARDWARE: hardware_bringup_handoff.md
 * accepted this subsystem as low-risk without an independent test,
 * since it's a simple digital output with no shared bus involved. This
 * driver's own init function includes a short, gentle pulse specifically
 * so the very first time it runs on real hardware, you get direct,
 * unambiguous confirmation the motor and its transistor circuit
 * actually work — the equivalent of the "does it respond" checks every
 * other sensor in this project got before being trusted.
 */

esp_err_t vibration_driver_init(void);

// Turns the motor on for duration_ms milliseconds, then off. Blocks
// the calling task for that duration (this project's use of it is
// brief pulses, not long-running patterns, so a blocking call keeps
// the calling code simple).
void vibration_pulse(uint32_t duration_ms);

void vibration_set(bool on);
