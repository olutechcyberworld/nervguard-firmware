#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

/*
 * Shared I2C bus.
 *
 * PLAIN-LANGUAGE SUMMARY:
 * Two sensors (MAX30102 heart rate, and the upcoming MPU6500 movement
 * sensor) are wired to the same two physical pins, per
 * circuit_design_handoff.md (GPIO8 = SDA, GPIO9 = SCL). Only one piece
 * of code is allowed to "own" that physical connection. This component
 * is that one owner. Any driver that needs to talk to a sensor on this
 * bus calls i2c_bus_get_handle() to get access to it, rather than trying
 * to set up its own separate connection on the same pins, which would
 * conflict.
 */

// Sets up the shared I2C connection. Safe to call more than once; only
// the first call actually does anything.
esp_err_t i2c_bus_init(void);

// Returns the shared bus handle, for a sensor driver to add itself to.
// Returns NULL if i2c_bus_init() has not been called yet or failed.
i2c_master_bus_handle_t i2c_bus_get_handle(void);
