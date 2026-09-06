#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "feature_extraction.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * BLE GATT server — implements the full contract locked in
 * ble_gatt_contract_handoff.md.
 *
 * PLAIN-LANGUAGE SUMMARY:
 * This is what lets the phone app actually talk to the device. It
 * advertises over Bluetooth, tracks which of seven states the device
 * is in (not calibrated yet, calibrating, idle, actively monitoring,
 * not being worn, an error, or mid-reset), and pushes fresh data out to
 * the app whenever it's ready — inference results, raw feature values,
 * and state changes.
 *
 * REQUIRES ONE MANUAL PROJECT-LEVEL CHANGE THIS CODE CANNOT MAKE:
 * NimBLE (ESP-IDF's Bluetooth Low Energy stack) must be enabled in your
 * project's sdkconfig before this will build. Run `idf.py menuconfig`,
 * go to Component config -> Bluetooth, enable Bluetooth, and select
 * "NimBLE - BLE only" as the host stack. See the CMakeLists.txt in this
 * folder for the exact component this depends on.
 */

esp_err_t ble_gatt_server_init(void);

// Call this once per feature_extraction window (i.e. whenever
// feature_extraction_get_latest() returns true) — pushes the 53-byte
// Feature Vector notification if the app is currently subscribed, and
// feeds the calibration accumulator if the device is in CALIBRATING
// state.
void ble_gatt_server_notify_features(const feature_vector_t *features, uint8_t window_sequence);

// Call this once per completed inference — pushes the 3-byte Inference
// Output notification if the device is in MONITORING state.
void ble_gatt_server_notify_inference(int stress_class, float probability, uint8_t window_sequence);

// Call periodically (e.g. once per feature_extraction_tick, using the
// same 4Hz temperature value already available) — drives wear
// detection (temp_mean < 28.0C / > 30.0C hysteresis, per the locked
// contract).
void ble_gatt_server_update_wear_detection(float current_temp_c);

// Puts the device into the ERROR state (0x05) with the given error code,
// per the state and error tables locked in ble_gatt_contract_handoff.md,
// and notifies any subscribed app immediately. Safe to call at any point
// after ble_gatt_server_init() has returned ESP_OK, including before any
// BLE central has connected — the notification is simply skipped if
// nobody is subscribed yet, and a subsequent connection's initial READ
// of the Device State characteristic will still return ERROR correctly.
//
// Recognised codes (see error_code_t in ble_gatt_server.c and the
// contract's Error Code Reference table):
//   0x01  I2C bus failure           (MAX30102 or MPU6500/MPU6050)
//   0x02  1-Wire failure            (DS18B20)
//   0x03  EDA ADC failure
//   0x04  TFLite allocation failure
//   0x05  NVS failure
//   0x06  Persistent signal fault
// There is currently no locked contract code for a vibration motor,
// RGB LED, or feature-extraction init failure — main.c logs those
// distinctly rather than forcing them through one of the codes above.
// This is a real gap, not an oversight to paper over; see the
// accompanying handoff note for the recommended contract revision.
void ble_gatt_server_set_error(uint8_t error_code);

#ifdef __cplusplus
}
#endif