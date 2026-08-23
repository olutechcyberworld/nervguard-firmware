#include "eda_driver.h"

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "eda_driver";

// GPIO4 = ADC1 Channel 3 on the ESP32-S3, locked in
// circuit_design_handoff.md, do not change without a formal circuit
// revision.
#define EDA_ADC_UNIT    ADC_UNIT_1
#define EDA_ADC_CHANNEL ADC_CHANNEL_3
#define EDA_ADC_ATTEN   ADC_ATTEN_DB_12  // covers the full 0–3.3V range

// --- Calibration table. ---
//
// These are REAL MEASURED POINTS from hardware_bringup_handoff.md, not
// a theoretical formula. That document explains why: the LM358's exact
// input bias current isn't confirmed for the specific installed part,
// so an analytic formula would be guessing at a number nobody has
// actually measured. This table sidesteps that entirely by using only
// what was directly measured on the real circuit.
//
// KNOWN GAP: the jump between 100kΩ and 470kΩ is wide and still needs a
// 220kΩ point. The table also doesn't yet reach down to 50kΩ. Add rows
// here — in voltage order, low to high — as those measurements become
// available. Nothing else in this file needs to change to support that.
typedef struct {
    float voltage;
    float resistance_ohms;
} eda_cal_point_t;

static const eda_cal_point_t s_cal_table[] = {
    { .voltage = 0.62f, .resistance_ohms = 1000000.0f },
    { .voltage = 0.66f, .resistance_ohms = 470000.0f  },
    { .voltage = 1.46f, .resistance_ohms = 100000.0f  },
};
#define CAL_TABLE_SIZE (sizeof(s_cal_table) / sizeof(s_cal_table[0]))

// --- Noise rejection. ---
//
// Confirmed on real hardware: a finger bridging the electrodes can pick
// up ambient electrical noise fast enough to swing the reading by
// hundreds of percent within a single 10ms sample. Real skin
// conductance cannot physically change that fast — genuine skin
// conductance responses rise over roughly 1–3 SECONDS, meaning even a
// fast real change should only move by a small fraction of a percent
// from one 10ms sample to the next.
//
// 15% was chosen as a threshold comfortably above any plausible single-
// sample real change, while still easily catching the noise spikes
// actually observed on this board (which were changing by 300–600%+
// between consecutive samples).
#define EDA_MAX_STEP_CHANGE_PERCENT 15.0f

#define RING_BUFFER_SIZE 400  // 4 seconds of samples at 100Hz

static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t s_cali_handle = NULL;
static bool s_have_calibration = false;

static SemaphoreHandle_t s_buffer_mutex = NULL;
static eda_sample_t s_ring_buffer[RING_BUFFER_SIZE];
static size_t s_ring_head = 0;
static size_t s_ring_count = 0;
static eda_sample_t s_last_sample = {0};
static bool s_have_sample = false;

// Tracks the last reading that PASSED the noise filter, separately from
// s_last_sample (which is simply the most recent output, artifact or
// not). This is what new readings get compared against.
static float s_last_trusted_conductance_us = -1.0f;  // -1 = none yet

// Tracks the previous RAW reading (before any filtering), and how many
// consecutive raw readings in a row have agreed with each other. This
// is what lets the filter recognise a genuine step change (several
// readings in a row landing in a new, consistent place) rather than
// getting stuck comparing forever against an increasingly stale
// trusted value. See the "genuine step change" note in
// voltage_to_resistance()'s caller below for the full reasoning.
static float s_last_raw_conductance_us = -1.0f;
static int s_consecutive_agreeing_count = 0;
#define EDA_STEP_CHANGE_CONFIRM_COUNT 3

// --- Converts a measured voltage into a skin resistance value, using
//     the real calibration table above. ---
//
// PLAIN-LANGUAGE SUMMARY:
// If the voltage falls between two points we've actually measured, this
// draws a straight line between them (in "log-resistance space" — the
// same interpolation method the bring-up session's own prototype used,
// since resistance changes are naturally more linear on a logarithmic
// scale than a plain one) and reads the resistance off that line. If
// the voltage falls outside every point we've measured, it extrapolates
// past the nearest edge of the table instead of just clamping to it —
// but flags the result as uncalibrated, since that's a genuine guess,
// not a measurement.
static float voltage_to_resistance(float voltage, bool *out_calibrated)
{
    *out_calibrated = (voltage >= s_cal_table[0].voltage &&
                        voltage <= s_cal_table[CAL_TABLE_SIZE - 1].voltage);

    // Find the two calibration points this voltage sits between (or,
    // if it's outside the table entirely, the two nearest points to
    // extrapolate from).
    size_t lower = 0;
    size_t upper = CAL_TABLE_SIZE - 1;
    for (size_t i = 0; i < CAL_TABLE_SIZE - 1; i++) {
        if (voltage >= s_cal_table[i].voltage && voltage <= s_cal_table[i + 1].voltage) {
            lower = i;
            upper = i + 1;
            break;
        }
    }
    if (voltage < s_cal_table[0].voltage) {
        lower = 0; upper = 1;
    } else if (voltage > s_cal_table[CAL_TABLE_SIZE - 1].voltage) {
        lower = CAL_TABLE_SIZE - 2; upper = CAL_TABLE_SIZE - 1;
    }

    float v_lo = s_cal_table[lower].voltage;
    float v_hi = s_cal_table[upper].voltage;
    float log_r_lo = log10f(s_cal_table[lower].resistance_ohms);
    float log_r_hi = log10f(s_cal_table[upper].resistance_ohms);

    float fraction = (voltage - v_lo) / (v_hi - v_lo);
    float log_r = log_r_lo + (log_r_hi - log_r_lo) * fraction;

    return powf(10.0f, log_r);
}

static void push_sample(eda_sample_t sample)
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

// --- Background task: samples the EDA voltage at a steady 100Hz,
//     matching the acquisition rate specified in
//     chat2_firmware_handoff.md. ---
static void eda_reading_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));

        int raw = 0;
        esp_err_t err = adc_oneshot_read(s_adc_handle, EDA_ADC_CHANNEL, &raw);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read EDA ADC: %s", esp_err_to_name(err));
            continue;
        }

        int millivolts = 0;
        if (s_have_calibration) {
            adc_cali_raw_to_voltage(s_cali_handle, raw, &millivolts);
        } else {
            // Fallback if calibration setup failed on this chip — a
            // rough approximation, not a substitute for real
            // calibration. Logged clearly at init time if this path is
            // ever taken.
            millivolts = (int)((float)raw / 4095.0f * 3300.0f);
        }

        float voltage = (float)millivolts / 1000.0f;

        bool calibrated = false;
        float resistance_ohms = voltage_to_resistance(voltage, &calibrated);
        float raw_conductance_us = (resistance_ohms > 0.0f) ? (1000000.0f / resistance_ohms) : 0.0f;

        // --- Noise filter ---
        //
        // BUG FIX (found via real-hardware testing): the first version
        // of this filter only ever compared a new reading against the
        // ORIGINAL trusted value. If the real signal genuinely moved —
        // for example, a finger actually landing on the sensor — every
        // subsequent real reading still looked like too big a jump
        // from that now-stale trusted value, so the filter rejected
        // everything forever and froze on a multi-second-old number.
        // Confirmed on real hardware: stretches of 27/27 rejected
        // samples lasting several seconds straight.
        //
        // The fix: also track whether consecutive RAW readings agree
        // with EACH OTHER, not just with the old trusted value. Real
        // electrical noise is inconsistent from one 10ms sample to the
        // next. A genuine change moves to a new value and then holds
        // steady there. So if several raw readings in a row land in
        // roughly the same new place, that consistency itself is proof
        // the world genuinely changed — not noise — and the filter
        // catches up rather than rejecting indefinitely.
        bool artifact_rejected = false;
        float output_conductance_us = raw_conductance_us;

        if (s_last_trusted_conductance_us < 0.0f) {
            // First-ever sample — nothing to compare against yet.
            s_last_trusted_conductance_us = raw_conductance_us;
            s_last_raw_conductance_us = raw_conductance_us;
        } else {
            float change_vs_trusted = fabsf(raw_conductance_us - s_last_trusted_conductance_us) /
                                       s_last_trusted_conductance_us * 100.0f;

            if (change_vs_trusted <= EDA_MAX_STEP_CHANGE_PERCENT) {
                // Close enough to the trusted value — accept normally.
                s_last_trusted_conductance_us = raw_conductance_us;
                s_consecutive_agreeing_count = 0;
            } else {
                // Far from the trusted value. Does it at least agree
                // with the immediately preceding raw reading?
                float change_vs_last_raw = fabsf(raw_conductance_us - s_last_raw_conductance_us) /
                                            s_last_raw_conductance_us * 100.0f;

                if (change_vs_last_raw <= EDA_MAX_STEP_CHANGE_PERCENT) {
                    // Consistent with the last raw reading — this could
                    // be the start of a genuine step change.
                    s_consecutive_agreeing_count++;
                    if (s_consecutive_agreeing_count >= EDA_STEP_CHANGE_CONFIRM_COUNT) {
                        // Several readings in a row agree with each
                        // other in this new place — trust it.
                        s_last_trusted_conductance_us = raw_conductance_us;
                        s_consecutive_agreeing_count = 0;
                    } else {
                        artifact_rejected = true;
                        output_conductance_us = s_last_trusted_conductance_us;
                    }
                } else {
                    // Doesn't even agree with the last raw reading —
                    // genuinely inconsistent, almost certainly noise.
                    s_consecutive_agreeing_count = 0;
                    artifact_rejected = true;
                    output_conductance_us = s_last_trusted_conductance_us;
                }
            }
            s_last_raw_conductance_us = raw_conductance_us;
        }

        // resistance_ohms is recomputed from the OUTPUT conductance, so
        // the two fields in the sample always agree with each other,
        // even when the reading was held rather than fresh.
        float output_resistance_ohms = (output_conductance_us > 0.0f)
                                        ? (1000000.0f / output_conductance_us)
                                        : resistance_ohms;

        eda_sample_t sample = {
            .voltage = voltage,  // kept as actually measured, even when rejected — useful for diagnostics
            .resistance_ohms = output_resistance_ohms,
            .conductance_us = output_conductance_us,
            .calibrated = calibrated,
            .artifact_rejected = artifact_rejected,
        };

        push_sample(sample);
    }
}

esp_err_t eda_driver_init(void)
{
    s_buffer_mutex = xSemaphoreCreateMutex();
    if (s_buffer_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = EDA_ADC_UNIT,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_config, &s_adc_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set up the ADC unit: %s", esp_err_to_name(err));
        return err;
    }

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = EDA_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(s_adc_handle, EDA_ADC_CHANNEL, &chan_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure EDA ADC channel: %s", esp_err_to_name(err));
        return err;
    }

    // ADC calibration converts raw ADC counts into an accurate voltage,
    // correcting for the ESP32-S3's own chip-to-chip manufacturing
    // variation. Without this, raw-to-voltage conversion is a rough
    // approximation rather than a real measurement.
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = EDA_ADC_UNIT,
        .chan = EDA_ADC_CHANNEL,
        .atten = EDA_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_cali_create_scheme_curve_fitting(&cali_config, &s_cali_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ADC calibration unavailable on this chip (%s) — falling back to an "
                       "uncalibrated approximation. Readings will be less accurate.",
                  esp_err_to_name(err));
        s_have_calibration = false;
    } else {
        s_have_calibration = true;
    }

    BaseType_t ok = xTaskCreate(eda_reading_task, "eda_read", 3072, NULL, 5, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Could not start the background reading task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "EDA ready on GPIO4, 100Hz, calibration table has %u measured points "
                   "(known gap: 100kΩ–470kΩ, and below 100kΩ — see eda_driver.h)",
              (unsigned)CAL_TABLE_SIZE);
    return ESP_OK;
}

esp_err_t eda_read_samples(eda_sample_t *out_buf, size_t max_count, size_t *out_count)
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

bool eda_get_last_sample(eda_sample_t *out_sample)
{
    xSemaphoreTake(s_buffer_mutex, portMAX_DELAY);
    bool have_sample = s_have_sample;
    eda_sample_t sample = s_last_sample;
    xSemaphoreGive(s_buffer_mutex);

    if (!have_sample) {
        return false;
    }
    *out_sample = sample;
    return true;
}