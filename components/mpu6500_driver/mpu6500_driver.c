#include "mpu6500_driver.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "i2c_bus.h"

static const char *TAG = "mpu6500_driver";

// 7-bit I2C address. AD0 tied to GND (per circuit_design_handoff.md)
// selects 0x68. Confirmed against the real board in
// hardware_bringup_handoff.md (I2C bus scan) — address only; this
// driver is the first time actual accelerometer data is read from it.
#define MPU6500_I2C_ADDR 0x68

// --- Register addresses, per the MPU6500 register map. ---
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B  // 6 contiguous bytes: X, Y, Z, high byte then low byte each
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

// Confirmed directly against the register map before writing this
// driver. Two values are accepted here:
//   0x70 — MPU6500, the part specified in circuit_design_handoff.md
//   0x68 — MPU6050, an older sibling chip in the same family
// Real hardware evidence (this exact check, run on the real board)
// showed the installed part actually answers 0x68, meaning it's a
// genuine MPU6050, not the MPU6500 the circuit design specifies. This
// is a documented deviation between the design and the physical build,
// in the same spirit as other measured-vs-assumed mismatches already
// logged elsewhere in this project. It's accepted here because MPU6050
// and MPU6500 share the same register layout and scale factors for
// everything this driver uses.
#define WHO_AM_I_MPU6500 0x70
#define WHO_AM_I_MPU6050 0x68

// PWR_MGMT_1 = 0x00: wakes the chip up. It powers on with its "sleep"
// bit set by default, which would otherwise make every register read
// come back as stale, unchanging data.
#define PWR_MGMT_1_WAKE 0x00

// CONFIG: enables the sensor's own internal digital low-pass filter
// (DLPF_CFG = 3, roughly a 41Hz bandwidth), which smooths out
// high-frequency mechanical vibration/noise before we ever read the
// value, and fixes the sensor's own internal sample rate at 1kHz —
// required for the SMPLRT_DIV setting below to produce exactly 100Hz.
#define CONFIG_DLPF_VALUE 0x03

// SMPLRT_DIV: output rate = 1kHz / (1 + SMPLRT_DIV). A value of 9 here
// gives 1000 / (1 + 9) = 100Hz, matching the 100Hz acquisition rate
// specified in chat2_firmware_handoff.md for this sensor.
#define SMPLRT_DIV_VALUE 9

// ACCEL_CONFIG: selects ±4g full-scale range (bits [4:3] = 01). Chosen
// as a middle ground — wide enough to avoid clipping during normal
// wrist movement, without giving up more resolution than needed for a
// mostly-sedentary stress-monitoring use case.
#define ACCEL_CONFIG_4G 0b00001000

// The MPU6500's own real scale factor for ±4g range: 8192 raw counts
// per 1g. This is NOT the same number as the Empatica E4's 64-counts-
// per-g figure mentioned in ml_pipeline_documentation.md — that was a
// different, unrelated device. See the header file for the full note.
#define LSB_PER_G 8192.0f

#define RING_BUFFER_SIZE 400  // 4 seconds of samples at 100Hz

static i2c_master_dev_handle_t s_dev_handle = NULL;

static SemaphoreHandle_t s_buffer_mutex = NULL;
static mpu6500_sample_t s_ring_buffer[RING_BUFFER_SIZE];
static size_t s_ring_head = 0;
static size_t s_ring_count = 0;
static mpu6500_sample_t s_last_sample = {0};
static bool s_have_sample = false;

// --- Small I2C helpers, same pattern as the MAX30102 driver. ---

static esp_err_t write_register(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_transmit(s_dev_handle, buf, sizeof(buf), 100 /* ms timeout */);
}

static esp_err_t read_registers(uint8_t start_reg, uint8_t *out_buf, size_t len)
{
    return i2c_master_transmit_receive(s_dev_handle, &start_reg, 1, out_buf, len, 100 /* ms timeout */);
}

static void push_sample(mpu6500_sample_t sample)
{
    xSemaphoreTake(s_buffer_mutex, portMAX_DELAY);
    s_ring_buffer[s_ring_head] = sample;
    s_ring_head = (s_ring_head + 1) % RING_BUFFER_SIZE;
    if (s_ring_count < RING_BUFFER_SIZE) {
        s_ring_count++;
    } else {
        ESP_LOGW(TAG, "Sample buffer full — oldest unread sample was overwritten");
    }
    s_last_sample = sample;
    s_have_sample = true;
    xSemaphoreGive(s_buffer_mutex);
}

// --- Background task: samples the accelerometer at a steady 100Hz. ---
//
// PLAIN-LANGUAGE SUMMARY OF THIS LOOP:
// Unlike the heart rate sensor, this chip doesn't queue up multiple
// readings on its own — it just holds whatever its latest measurement
// is, ready to be read at any time. So this task simply reads it once
// every 10 milliseconds (100 times a second) and converts the result
// into g-units before storing it.
static void mpu6500_reading_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));

        uint8_t raw[6] = {0};
        esp_err_t err = read_registers(REG_ACCEL_XOUT_H, raw, sizeof(raw));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read accelerometer data: %s", esp_err_to_name(err));
            continue;
        }

        // Each axis is a 16-bit signed value, high byte first.
        int16_t x_raw = (int16_t)((raw[0] << 8) | raw[1]);
        int16_t y_raw = (int16_t)((raw[2] << 8) | raw[3]);
        int16_t z_raw = (int16_t)((raw[4] << 8) | raw[5]);

        mpu6500_sample_t sample = {
            .x_g = (float)x_raw / LSB_PER_G,
            .y_g = (float)y_raw / LSB_PER_G,
            .z_g = (float)z_raw / LSB_PER_G,
        };

        push_sample(sample);
    }
}

esp_err_t mpu6500_driver_init(void)
{
    s_buffer_mutex = xSemaphoreCreateMutex();
    if (s_buffer_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = i2c_bus_init();
    if (err != ESP_OK) {
        return err;
    }

    i2c_master_bus_handle_t bus = i2c_bus_get_handle();
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6500_I2C_ADDR,
        .scl_speed_hz = 400000,  // 400kHz Fast Mode, per circuit_design_handoff.md
    };
    err = i2c_master_bus_add_device(bus, &dev_config, &s_dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add MPU6500 to the I2C bus: %s", esp_err_to_name(err));
        return err;
    }

    // Confirm this is genuinely an MPU6500 before configuring it — same
    // "measure, don't assume" discipline used throughout this project.
    // This is also this sensor's first real data check: hardware
    // bring-up only confirmed its address responds, not that real
    // register reads work correctly.
    uint8_t who_am_i = 0;
    err = read_registers(REG_WHO_AM_I, &who_am_i, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MPU6500 WHO_AM_I register: %s", esp_err_to_name(err));
        return err;
    }
    if (who_am_i != WHO_AM_I_MPU6500 && who_am_i != WHO_AM_I_MPU6050) {
        ESP_LOGE(TAG, "Unexpected WHO_AM_I: got 0x%02X, expected 0x%02X (MPU6500) or 0x%02X (MPU6050). "
                       "Check wiring, AD0 strapping, and I2C address.",
                  who_am_i, WHO_AM_I_MPU6500, WHO_AM_I_MPU6050);
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (who_am_i == WHO_AM_I_MPU6050) {
        ESP_LOGW(TAG, "Detected an MPU6050, not the MPU6500 specified in circuit_design_handoff.md. "
                       "Proceeding — register layout and scale factors are compatible for accelerometer "
                       "use. This is a documented design-vs-build deviation, not a firmware fault.");
    }

    write_register(REG_PWR_MGMT_1, PWR_MGMT_1_WAKE);
    write_register(REG_CONFIG, CONFIG_DLPF_VALUE);
    write_register(REG_SMPLRT_DIV, SMPLRT_DIV_VALUE);
    write_register(REG_ACCEL_CONFIG, ACCEL_CONFIG_4G);

    BaseType_t ok = xTaskCreate(mpu6500_reading_task, "mpu6500_read", 3072, NULL, 5, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Could not start the background reading task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "%s ready (WHO_AM_I confirmed 0x%02X), ±4g range, 100Hz",
              (who_am_i == WHO_AM_I_MPU6050) ? "MPU6050" : "MPU6500", who_am_i);
    return ESP_OK;
}

esp_err_t mpu6500_read_samples(mpu6500_sample_t *out_buf, size_t max_count, size_t *out_count)
{
    xSemaphoreTake(s_buffer_mutex, portMAX_DELAY);

    size_t to_read = (s_ring_count < max_count) ? s_ring_count : max_count;
    size_t read_start = (s_ring_head + RING_BUFFER_SIZE - s_ring_count) % RING_BUFFER_SIZE;

    for (size_t i = 0; i < to_read; i++) {
        out_buf[i] = s_ring_buffer[(read_start + i) % RING_BUFFER_SIZE];
    }
    s_ring_count -= to_read;

    xSemaphoreGive(s_buffer_mutex);

    *out_count = to_read;
    return ESP_OK;
}

bool mpu6500_get_last_sample(mpu6500_sample_t *out_sample)
{
    xSemaphoreTake(s_buffer_mutex, portMAX_DELAY);
    bool have_sample = s_have_sample;
    mpu6500_sample_t sample = s_last_sample;
    xSemaphoreGive(s_buffer_mutex);

    if (!have_sample) {
        return false;
    }
    *out_sample = sample;
    return true;
}