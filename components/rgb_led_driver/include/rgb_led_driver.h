#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/*
 * RGB status LED driver.
 *
 * PLAIN-LANGUAGE SUMMARY:
 * Three plain digital outputs (GPIO12=Red, GPIO13=Green, GPIO14=Blue),
 * per circuit_design_handoff.md's Subsystem 6, common-cathode with
 * current-limiting resistors already on the board. This driver just
 * turns each colour channel fully on or off — no brightness control
 * (PWM/dimming), since nothing in this project's design calls for it.
 *
 * A handful of named colours are provided as simple status indicators
 * for later use (e.g. by the calibration or intervention flow) —
 * these are a reasonable starting set, not a locked design decision;
 * add more as the app/firmware interaction design calls for them.
 *
 * NOT YET EXERCISED ON REAL HARDWARE: same as the vibration motor —
 * accepted as low-risk in hardware_bringup_handoff.md, but genuinely
 * untested until this driver's own init confirms it.
 */

typedef enum {
    LED_OFF = 0,
    LED_RED,
    LED_GREEN,
    LED_BLUE,
    LED_WHITE,
    LED_YELLOW,   // red + green — a reasonable "calibrating" indicator
    LED_PURPLE,   // red + blue
    LED_CYAN,     // green + blue
} led_color_t;

esp_err_t rgb_led_driver_init(void);

void rgb_led_set(led_color_t color);
