#include "feature_extraction.h"

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <float.h>
#include "esp_log.h"
#include "esp_timer.h"

#include "ds18b20_driver.h"
#include "max30102_driver.h"
#include "mpu6500_driver.h"
#include "eda_driver.h"

static const char *TAG = "feature_extraction";

// --- Batch summary accessors. ---
//
// PLAIN-LANGUAGE SUMMARY OF WHY THESE EXIST:
// Real hardware logging in this session confirmed that
// max30102_read_ir_samples(), eda_read_samples(), and
// mpu6500_read_samples() all CONSUME from a shared ring buffer — each
// call removes whatever it returns, permanently, for every caller.
// main.c was calling all three a second time, purely to print its own
// console summary, immediately before feature_extraction_tick() called
// them again for real feature computation. Whichever call ran first
// took most of that tick's samples, leaving the second call starved —
// confirmed directly via [HRV-DIAG-BATCH] logging showing ~2 samples
// reaching feed_heart_rate() on ticks where main.c's own line reported
// 24-30. This silently corrupted every one of the 13 locked features
// this entire session, not just HRV, since eda_driver.c and
// mpu6500_driver.c use the identical destructive-read ring buffer
// pattern as max30102_driver.c.
//
// The fix: feature_extraction.c is now the SOLE caller of all three
// sensor read functions. main.c gets everything it needs for its own
// console line from these three summary structs instead, computed
// here at the same moment the real feature-extraction read happens, so
// there is exactly one read of each sensor per tick, not two.
//
// The three struct typedefs (hr_batch_summary_t, accel_batch_summary_t,
// eda_batch_summary_t) now live in feature_extraction.h, since main.c
// needs them too. Only the static storage and accessor bodies stay here.
static hr_batch_summary_t s_last_hr_summary = {0};
static accel_batch_summary_t s_last_accel_summary = {0};
static eda_batch_summary_t s_last_eda_summary = {0};

// Each returns true if this tick's read actually produced at least one
// sample (matching the "count > 0" check main.c used to do directly
// against the driver). out is always fully written either way, with
// count = 0 and other fields zeroed on a false return, so a caller
// that doesn't check the return value still gets a safe, well-defined
// struct rather than stale or uninitialised data.
bool feature_extraction_get_last_hr_batch(hr_batch_summary_t *out)
{
    *out = s_last_hr_summary;
    return s_last_hr_summary.count > 0;
}

bool feature_extraction_get_last_accel_batch(accel_batch_summary_t *out)
{
    *out = s_last_accel_summary;
    return s_last_accel_summary.count > 0;
}

bool feature_extraction_get_last_eda_batch(eda_batch_summary_t *out)
{
    *out = s_last_eda_summary;
    return s_last_eda_summary.count > 0;
}

// Main loop calls feature_extraction_tick() every 250ms (4Hz) — this
// file assumes that cadence throughout.
#define TICK_MS       250
#define WINDOW_TICKS  240   // 60 seconds / 250ms
#define STEP_TICKS    20    // 5 seconds / 250ms

// --- EDA and TEMP windows: exactly one averaged value per 250ms tick,
//     which is exactly 4Hz — no separate downsampling step needed. ---
static float s_eda_buf[WINDOW_TICKS];
static float s_temp_buf[WINDOW_TICKS];

// Parallel ring buffer to s_eda_buf, same index, same lifecycle: each
// slot holds the FRACTION of that tick's raw EDA samples that were
// either extrapolated (outside the 3-point calibration table) or
// noise-rejected (artifact_rejected), rather than the averaged
// conductance value itself. This exists purely to let calibration
// reject a noisy window — see feed_eda() below and the validity check
// in ble_gatt_server.c's feed_calibration_window(), which is the actual
// consumer. It intentionally does NOT enter feature_vector_to_array()'s
// 13-element output and therefore never touches the BLE wire format
// locked in ble_gatt_contract_handoff.md.
static float s_eda_reject_frac_buf[WINDOW_TICKS];

// --- ACC window at 32Hz: 8 samples per 250ms tick (approximately —
//     see the note in feed_accel() below about why this is a
//     simplification, not a proper anti-alias filter). ---
#define ACC_SAMPLES_PER_TICK 8
#define ACC_WINDOW_SIZE (WINDOW_TICKS * ACC_SAMPLES_PER_TICK)  // 1920
static float s_acc_mag_buf[ACC_WINDOW_SIZE];
static size_t s_acc_write_index = 0;

// --- RR intervals (for HRV), stored with real timestamps so we can
//     pick out "whatever fell in the last 60 seconds" at each step. ---
#define RR_BUFFER_SIZE 300  // generous — even 180bpm for 60s is only 180 beats
typedef struct {
    float rr_ms;
    int64_t timestamp_us;
} rr_entry_t;
static rr_entry_t s_rr_buf[RR_BUFFER_SIZE];
static size_t s_rr_write_index = 0;
static size_t s_rr_count = 0;

// --- Simple online peak detector for the raw IR stream. ---
//
// PLAIN-LANGUAGE SUMMARY:
// This looks at each new raw IR sample as it arrives and tries to spot
// the moment a heartbeat's pulse peak just passed. It keeps a rolling
// sense of "how high and low has the signal been recently" to set a
// threshold that adapts as the baseline drifts, and it refuses to count
// a second peak too soon after the last one (a "refractory period"),
// which stops noise right around a real peak from being counted twice.
//
// This is a first-pass implementation, per chat2_firmware_handoff.md's
// own recommendation ("simple threshold-based peak detector with
// refractory period"). It has not yet been tuned against a real
// clean-vs-noisy waveform comparison on this exact board — that
// tuning is a carried-forward item once real HRV output can be
// compared against a reference (e.g. a pulse oximeter) during
// integration testing.
#define PEAK_REFRACTORY_SAMPLES 40  // 400ms at 100Hz -> caps detection at 150bpm
#define PEAK_ROLLING_WINDOW 100     // ~1 second at 100Hz, for adaptive min/max

// BUG FIX: this used to be a single running min/max that snapped
// instantly to any new extreme and only relaxed back at a flat 1
// count/sample, rather than an actual rolling window over the last
// PEAK_ROLLING_WINDOW samples (that constant existed but was never
// used). Against a genuinely drifting baseline, real hardware showed
// this let the min/max range balloon well past the true local pulse
// amplitude, dragging the 40%-of-range threshold up until genuine
// heartbeat peaks stopped crossing it at all — HRV features stayed
// sentineled at -1.0 for minutes at a time despite a clearly strong,
// well-formed raw pulsatile signal throughout. A real circular buffer
// of the last 100 raw samples, rescanned for min/max on every new
// sample, tracks the true local range regardless of how fast the
// baseline itself drifts. At 100 Hz this is 100 comparisons per
// sample, entirely trivial for this chip, so there is no performance
// reason to keep the cheaper approximation now that it is known not
// to work.
static float s_ir_window[PEAK_ROLLING_WINDOW];
static size_t s_ir_window_index = 0;
static size_t s_ir_window_count = 0;

// --- SRMAC detector (Attempt 7) — PROMOTED TO PRIMARY ---
//
// PLAIN-LANGUAGE SUMMARY:
// This is a SECOND, independent peak detector, originally added
// alongside the existing threshold+hysteresis one above purely for
// real-hardware comparison (logging only, no effect on real HRV
// output). CONFIRMED and PROMOTED: two real-hardware captures, one
// with the device not worn and one worn, ran both detectors
// simultaneously against the identical signal. Not worn: the legacy
// detector fired repeatedly at 394-446ms, noise tripping its own
// refractory floor, while this one fired only once in the same
// episode. Worn: 18 consecutive intervals from this detector ranged
// 953-1317ms, clustering around a coherent ~53bpm resting rate, while
// the legacy detector, on the exact same real signal, still spread
// 435-1641ms and still failed the 30-clean-interval bar. This
// detector's output now feeds s_rr_buf and compute_hrv_features()
// directly (see the doc comment on its ROI-close block below). The
// legacy detector is kept running and logging as an independent
// cross-check, not disconnected, in case a real calibration attempt's
// results ever need a second detector's numbers on the same capture.
//
// WHY: real-hardware evidence this session (tight pairwise-sum
// clustering in [HRV-DIAG-SEQ] logs) confirmed intermittent double-
// detection on the dicrotic notch, a small, real, physiologically
// normal secondary bump on the downslope after the true systolic peak.
// The hysteresis fix (Attempt 6) resolved that SPECIFIC clean pattern,
// but a messier, less structured spread of RR intervals persisted
// afterward, suggesting a shape-based detector may do better than a
// single static amplitude threshold plus a re-arm gate.
//
// METHOD: Smoothed Recursive Moving Average Crossover (SRMAC), a
// peer-reviewed, embedded-oriented PPG peak detector (Machado et al.,
// 2023, arXiv:2312.10013), reporting 0.9937 average precision and
// 0.9968 average recall across 22 subjects (healthy and COPD patients,
// at rest, walking, and in recovery), specifically designed to reject
// dicrotic-notch false positives that a simple crossover of two moving
// averages alone lets through. Three exponentially-weighted moving
// averages: a fast one and a slow one, whose difference rises sharply
// on a genuine systolic upstroke; a third smooths that difference to
// filter out the smaller, slower ripple a dicrotic notch produces. A
// Region Of Interest (ROI) is any stretch where the smoothed difference
// exceeds a threshold; the detected peak is the true local maximum of
// the raw (baseline-removed) signal within that ROI, not simply the
// first sample to cross a line.
//
// PARAMETER CAVEAT — the source paper optimized its four parameters
// (alpha_fast, alpha_slow, alpha_cross, threshold) against a 200Hz
// signal from a different sensor, after its own bandpass filtering and
// on its own normalized amplitude scale. None of that transfers
// directly to raw 100Hz MAX30102 IR counts. The values below are a
// first, reasoned starting point — alpha_fast/slow/cross chosen so
// their time constants (1 / (1-alpha) samples) sit at roughly
// 100ms/500ms/90ms, physiologically plausible relative to a systolic
// upstroke and one cardiac cycle. Real hardware has now confirmed these
// specific starting values produce a coherent result on this exact
// board (both captures above); they are validated for THIS hardware as
// tested, but still an unvalidated starting point for any future
// hardware revision, sensor swap, or different resting heart rate range
// this hasn't been exercised against yet — revisit if HRV output looks
// unreliable outside a calm, resting scenario. The threshold is
// expressed as a fraction of the same self-calibrating rolling min/max
// range the legacy detector already computes each sample, rather than a
// second hand-tuned fixed number with no real data behind it — but that
// fraction (5%) has now produced coherent results on real hardware
// across two captures, not merely a theoretical starting point.
#define SRMAC_ALPHA_FAST      0.90f  // ~10 samples = ~100ms time constant
#define SRMAC_ALPHA_SLOW      0.98f  // ~50 samples = ~500ms time constant
#define SRMAC_ALPHA_CROSS     0.88f  // ~8.3 samples = ~83ms, smooths short spurious ROI
#define SRMAC_THRESHOLD_FRACTION 0.05f  // fraction of rolling (recent_max - recent_min)

static float s_srmac_e_fast = 0.0f;
static float s_srmac_e_slow = 0.0f;
static float s_srmac_e_cross = 0.0f;
static bool s_srmac_initialized = false;
static bool s_srmac_in_roi = false;
static float s_srmac_roi_peak_value = 0.0f;
static int64_t s_srmac_roi_peak_time_us = 0;
static int64_t s_srmac_last_peak_time_us = 0;
static bool s_srmac_have_last_peak = false;

#define SRMAC_DIAG_LOG_EVERY_N_SAMPLES 100
static int s_srmac_diag_counter = 0;

static float s_ir_prev_sample = 0.0f;
static bool s_ir_rising = false;
static int s_refractory_countdown = 0;
static int64_t s_last_peak_time_us = 0;
static bool s_have_last_peak = false;

// FIX — Attempt 6. Real-hardware [HRV-DIAG-SEQ] logging confirmed the
// double-detection mechanism directly, via arithmetic rather than just
// pattern-matching: consecutive short "candidate" RR intervals summed
// in tight, repeatable clusters (e.g. 584+511=1095, 561+449=1010,
// 490+604=1094, 446+725=1171 within one window; 697+461=1158,
// 470+430=900, 678+422=1100, 457+526=983 within another) — all
// converging on the same true ~900-1170ms beat length, with individual
// unsplit intervals (850ms, 856ms) landing right inside that same
// band. No interval anywhere near 1800-2300ms (the signature a genuine
// MISSED beat would leave) appeared in either window, which rules that
// half of the original hypothesis out entirely. This is specifically a
// single real heartbeat being detected TWICE.
//
// The likely mechanism: a normal PPG waveform has a "dicrotic notch" —
// a small secondary bump on the downslope after the true systolic
// peak, a real physiological feature, not noise. The rising-then-
// falling-above-threshold rule below has no way to tell a shallow
// notch apart from a genuinely new beat; it only checks amplitude
// against a static-ish threshold, not shape.
//
// REJECTED FIX: widening PEAK_REFRACTORY_SAMPLES. The false intervals
// measured 420-730ms; suppressing those would need a refractory period
// of roughly 700-750ms, capping detection at ~80-85bpm. That would
// look fine on THIS resting log and then silently break exactly when
// this device matters most: WESAD's own TSST stress condition routinely
// produces 100-140bpm (430-600ms RR), which a refractory period tuned
// to this resting log's echoes would discard as if it were the
// artifact — trading a visible bug for an invisible, stress-blind one.
//
// ADOPTED FIX: hysteresis re-arming, tied to the same self-calibrating
// rolling min/max the primary threshold already uses (rather than a
// second, separately hand-tuned constant with no real data behind it
// yet). After a peak fires, no new peak may be detected until the
// signal first falls back down to at least the MIDPOINT of the recent
// min/max range. A genuine new heartbeat's full-amplitude downstroke
// clears this easily. A shallow dicrotic notch, by definition, does
// not — it happens on the way down from the true peak, before the
// signal has fully returned to baseline. This is a shape check, not a
// timing check, so unlike widening the refractory period it does not
// impose an artificial ceiling on detectable heart rate: a genuine
// fast beat during real stress still swings through the full
// min/max range and re-arms normally.
static bool s_peak_armed = true;

// Slow exponential-moving-average baseline tracker, subtracted from the
// raw signal before peak detection. A properly-implemented rolling
// min/max (added above) still measures max-min over its window, which
// captures genuine drift happening WITHIN that same window just as
// much as real oscillation — real hardware confirmed this: a baseline
// climbing at roughly 90-100 counts/second, comparable to or larger
// than the ~40-268 count genuine pulse amplitude, kept dragging the
// threshold around even with a correctly-sized window. Removing the
// slow-moving component before thresholding, a standard PPG technique,
// addresses the actual cause rather than compensating for it after the
// fact. BASELINE_EMA_ALPHA is deliberately small so its ~ (1 / (100Hz *
// alpha)) second time constant is long compared to a heartbeat period
// (roughly 0.5-1s even at a fast heart rate) — this tracks slow drift
// without attenuating the pulse itself. 0.002 (~5s time constant) is a
// first, reasoned value, not yet validated against a reference device;
// revisit alongside the peak detector's own known "not yet tuned"
// status if real HRV data still looks unreliable after this change.
#define BASELINE_EMA_ALPHA 0.002f
static float s_ir_baseline_ema = 0.0f;
static bool s_ir_baseline_initialized = false;

// TEMPORARY DIAGNOSTIC — Attempt 4, kept through Attempt 6 pending
// confirmation this fix actually resolves HRV on real hardware. Prior
// sessions confirmed (via candidate_count in compute_hrv_features())
// that peaks ARE being found, but that every one of them falls outside
// the 300-2000ms plausibility bound, at intervals of 7.5-11.3 seconds
// rather than the ~0.6-1.0s a real 60-100bpm pulse would produce. Back-calculating from
// the raw IR batch logs already gives strong circumstantial evidence:
// the observed baseline drift rate (roughly 170-230 counts/sec) times
// the EMA's own ~5s time constant (1 / (BASELINE_EMA_ALPHA * ~100Hz))
// predicts a steady-state tracking-lag of roughly 850-1150 counts,
// which matches the logged ac_value of 1192.8 almost exactly — well
// above the ~50-268 count genuine pulsatile amplitude confirmed during
// bring-up. This log makes that inference direct instead of derived:
// printing raw ir_value alongside baseline_ema and ac_value, throttled
// to roughly once per second (every 100th call at ~100Hz) so a full
// 60-second window is readable without flooding the log. If ac_value
// tracks the ramp-rate-times-time-constant relationship above rather
// than a repeating ~0.8-1s heartbeat periodicity, this confirms the
// EMA is chasing drift it cannot keep up with rather than isolating
// the pulse. Remove once the detrending fix is confirmed working.
#define BASELINE_DIAG_LOG_EVERY_N_SAMPLES 100
static int s_baseline_diag_counter = 0;

static void feed_ir_sample_for_peak_detection(uint32_t ir_value, int64_t timestamp_us)
{
    if (!s_ir_baseline_initialized) {
        // Seed on the very first sample rather than starting at 0, so
        // there is no large, fake initial "step" for the filter to
        // slowly climb out of.
        s_ir_baseline_ema = (float)ir_value;
        s_ir_baseline_initialized = true;
    } else {
        s_ir_baseline_ema += BASELINE_EMA_ALPHA * ((float)ir_value - s_ir_baseline_ema);
    }
    float ac_value = (float)ir_value - s_ir_baseline_ema;

    s_ir_window[s_ir_window_index] = ac_value;
    s_ir_window_index = (s_ir_window_index + 1) % PEAK_ROLLING_WINDOW;
    if (s_ir_window_count < PEAK_ROLLING_WINDOW) s_ir_window_count++;

    float recent_min = FLT_MAX;
    float recent_max = -FLT_MAX;
    for (size_t i = 0; i < s_ir_window_count; i++) {
        if (s_ir_window[i] < recent_min) recent_min = s_ir_window[i];
        if (s_ir_window[i] > recent_max) recent_max = s_ir_window[i];
    }

    float threshold = recent_min + (recent_max - recent_min) * 0.4f;
    float rearm_level = recent_min + (recent_max - recent_min) * 0.5f;

    // Re-arm check — see the "ADOPTED FIX" doc comment above
    // s_peak_armed's declaration. Once the signal has fallen back down
    // to at least the midpoint of the recent min/max range, a new peak
    // is allowed to be detected again. A genuine heartbeat's downstroke
    // clears this; a shallow dicrotic notch, occurring on the way down
    // from the true peak before the signal has fully returned, does
    // not. Checked every sample rather than only at the moment of a
    // falling edge, since the signal may sit below rearm_level for
    // several samples before the next real rise begins.
    if (!s_peak_armed && ac_value <= rearm_level) {
        s_peak_armed = true;
    }

    // TEMPORARY DIAGNOSTIC — Attempt 4, see the doc comment above
    // s_baseline_diag_counter's declaration for what this confirms.
    s_baseline_diag_counter++;
    if (s_baseline_diag_counter >= BASELINE_DIAG_LOG_EVERY_N_SAMPLES) {
        s_baseline_diag_counter = 0;
        ESP_LOGI(TAG, "[HRV-DIAG-TREND] ir_raw=%.1f baseline_ema=%.1f ac_value=%.1f threshold=%.1f",
                  (float)ir_value, s_ir_baseline_ema, ac_value, threshold);
    }

    // --- SRMAC detector (Attempt 7) — see the doc comment above
    // s_srmac_e_fast's declaration for the full method and caveats, and
    // the doc comment on its ROI-close block below for why it was
    // promoted to the PRIMARY detector feeding s_rr_buf and
    // compute_hrv_features(), based on a direct real-hardware
    // comparison against the legacy detector on the same signal. Runs
    // on the same ac_value (already baseline-drift-removed) as the
    // legacy detector, so it needs no bandpass filter of its own —
    // reusing work already done rather than duplicating it.
    if (!s_srmac_initialized) {
        s_srmac_e_fast = ac_value;
        s_srmac_e_slow = ac_value;
        s_srmac_e_cross = 0.0f;
        s_srmac_initialized = true;
    } else {
        s_srmac_e_fast += (1.0f - SRMAC_ALPHA_FAST) * (ac_value - s_srmac_e_fast);
        s_srmac_e_slow += (1.0f - SRMAC_ALPHA_SLOW) * (ac_value - s_srmac_e_slow);
        float diff = s_srmac_e_fast - s_srmac_e_slow;
        s_srmac_e_cross += (1.0f - SRMAC_ALPHA_CROSS) * (diff - s_srmac_e_cross);
    }
    float srmac_threshold = (recent_max - recent_min) * SRMAC_THRESHOLD_FRACTION;
    bool srmac_over_threshold = s_srmac_e_cross > srmac_threshold;

    if (!s_srmac_in_roi && srmac_over_threshold) {
        // Entering a new Region Of Interest — start tracking its true
        // local maximum, which is the eventual peak, not this sample.
        s_srmac_in_roi = true;
        s_srmac_roi_peak_value = ac_value;
        s_srmac_roi_peak_time_us = timestamp_us;
    } else if (s_srmac_in_roi) {
        if (ac_value > s_srmac_roi_peak_value) {
            s_srmac_roi_peak_value = ac_value;
            s_srmac_roi_peak_time_us = timestamp_us;
        }
        if (!srmac_over_threshold) {
            // ROI just closed — the tracked local maximum is the
            // detected peak.
            //
            // PROMOTED TO PRIMARY DETECTOR (Attempt 7, confirmed). Two
            // real-hardware captures, one with the device not worn and
            // one worn, both simultaneously ran this detector and the
            // legacy threshold+hysteresis one below against the exact
            // same signal. Not worn: legacy fired repeatedly at
            // 394-446ms (noise tripping its refractory floor); SRMAC
            // fired once in the same episode. Worn: 18 consecutive
            // SRMAC intervals ranged 953-1317ms (~53bpm, a coherent,
            // physiologically plausible resting rate) while the legacy
            // detector, on the identical real signal, still spread
            // 435-1641ms and still failed the 30-clean-interval bar.
            // SRMAC's RR interval is now what actually reaches
            // s_rr_buf and compute_hrv_features(). The legacy detector
            // below is kept running and logging, NOT feeding the real
            // buffer, as a continued independent cross-check — if a
            // real calibration attempt looks wrong, its numbers are
            // still there to compare against on the same capture.
            s_srmac_in_roi = false;
            if (s_srmac_have_last_peak) {
                float srmac_rr_ms = (float)(s_srmac_roi_peak_time_us - s_srmac_last_peak_time_us) / 1000.0f;
                s_rr_buf[s_rr_write_index] = (rr_entry_t){ .rr_ms = srmac_rr_ms, .timestamp_us = s_srmac_roi_peak_time_us };
                s_rr_write_index = (s_rr_write_index + 1) % RR_BUFFER_SIZE;
                if (s_rr_count < RR_BUFFER_SIZE) s_rr_count++;
                ESP_LOGI(TAG, "[SRMAC-DIAG] peak detected (PRIMARY), raw rr_ms=%.1f (e_cross=%.1f, threshold=%.1f)",
                          srmac_rr_ms, s_srmac_e_cross, srmac_threshold);
            }
            s_srmac_last_peak_time_us = s_srmac_roi_peak_time_us;
            s_srmac_have_last_peak = true;
        }
    }

    s_srmac_diag_counter++;
    if (s_srmac_diag_counter >= SRMAC_DIAG_LOG_EVERY_N_SAMPLES) {
        s_srmac_diag_counter = 0;
        ESP_LOGI(TAG, "[SRMAC-DIAG-TREND] e_fast=%.1f e_slow=%.1f e_cross=%.1f threshold=%.1f in_roi=%d",
                  s_srmac_e_fast, s_srmac_e_slow, s_srmac_e_cross, srmac_threshold, (int)s_srmac_in_roi);
    }

    bool now_rising = (ac_value > s_ir_prev_sample);

    if (s_refractory_countdown > 0) {
        s_refractory_countdown--;
    } else if (s_peak_armed && s_ir_rising && !now_rising && s_ir_prev_sample > threshold) {
        // Signal just turned from rising to falling, above threshold,
        // AND the detector is armed (see above) — this is a genuine
        // new peak, not an echo of the one already counted.
        //
        // DEMOTED TO CROSS-CHECK ONLY (Attempt 7). This detector no
        // longer writes to s_rr_buf — SRMAC above is now the primary
        // detector feeding compute_hrv_features(), per the real-
        // hardware comparison documented in the doc comment above its
        // ROI-close block. This one keeps running and logging purely
        // as an independent second opinion on the same real signal, in
        // case a real calibration attempt's results ever need a second
        // detector's numbers to compare against.
        if (s_have_last_peak) {
            float rr_ms = (float)(timestamp_us - s_last_peak_time_us) / 1000.0f;
            // TEMPORARY DIAGNOSTIC — remove once HRV availability is
            // confirmed working on real hardware. Two prior fix
            // attempts (a real rolling min/max window, then DC-baseline
            // removal) both failed to resolve HRV staying sentineled at
            // -1.0, with no direct visibility into why. This logs every
            // raw peak-to-peak interval BEFORE the 300-2000ms/20%-of-
            // median plausibility filter in compute_hrv_features() gets
            // a chance to discard it, so the next log tells us directly
            // whether the detector is finding zero peaks, or finding
            // peaks whose RR values are being filtered out downstream —
            // two different problems requiring two different fixes.
            ESP_LOGI(TAG, "[HRV-DIAG] peak detected (cross-check only), raw rr_ms=%.1f (threshold=%.1f, ac_value=%.1f, rearm_level=%.1f)",
                      rr_ms, threshold, ac_value, rearm_level);
        }
        s_last_peak_time_us = timestamp_us;
        s_have_last_peak = true;
        s_refractory_countdown = PEAK_REFRACTORY_SAMPLES;
        // Disarm — see the "ADOPTED FIX" doc comment above
        // s_peak_armed's declaration. No further peak can be detected
        // until the signal falls back to rearm_level, which the check
        // near the top of this function re-enables.
        s_peak_armed = false;
    }

    s_ir_rising = now_rising;
    s_ir_prev_sample = ac_value;
}

// --- Small stats helpers, shared by all four feature groups. ---

static void compute_mean_std(const float *data, size_t n, float *out_mean, float *out_std)
{
    if (n == 0) { *out_mean = 0; *out_std = 0; return; }
    float sum = 0;
    for (size_t i = 0; i < n; i++) sum += data[i];
    float mean = sum / (float)n;

    float sq_sum = 0;
    for (size_t i = 0; i < n; i++) {
        float d = data[i] - mean;
        sq_sum += d * d;
    }
    *out_mean = mean;
    *out_std = sqrtf(sq_sum / (float)n);
}

// Simple least-squares linear regression slope, treating sample index
// as the x-axis. Units: value-per-sample, matching how the original ML
// pipeline defined eda_slope and temp_slope.
static float compute_slope(const float *data, size_t n)
{
    if (n < 2) return 0.0f;
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
    for (size_t i = 0; i < n; i++) {
        float x = (float)i;
        float y = data[i];
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_xx += x * x;
    }
    float n_f = (float)n;
    float denominator = n_f * sum_xx - sum_x * sum_x;
    if (fabsf(denominator) < 1e-6f) return 0.0f;
    return (n_f * sum_xy - sum_x * sum_y) / denominator;
}

static int compare_floats(const void *a, const void *b)
{
    float fa = *(const float *)a, fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

// --- HRV computation from the RR interval buffer, matching the
//     artifact rejection rules locked in chat2_firmware_handoff.md. ---
static void compute_hrv_features(int64_t window_start_us, feature_vector_t *out)
{
    // BUG FIX: candidates, stage1, sorted, and clean were previously
    // declared as local (stack) arrays. Four separate float[300] arrays
    // in one stack frame is 4,800 bytes, which alone exceeds this
    // project's entire 3,584-byte main-task stack budget
    // (CONFIG_ESP_MAIN_TASK_STACK_SIZE in sdkconfig.esp32s3box) before
    // counting anything else on the call chain at this point --
    // feature_extraction_tick()'s own frame, qsort()'s internal
    // recursion below, or whatever app_main()'s loop already had live.
    // This produced an intermittent stack-overflow crash (Guru
    // Meditation, IllegalInstruction, PC == A0, backtrace CORRUPTED --
    // the signature of an overflowing stack overwriting a return
    // address before the function could use it) that only manifested
    // once genuine skin contact drove this function's deeper code
    // paths, particularly qsort's own stack usage, for the first time.
    // Moving these four arrays to static removes 4,800 bytes from every
    // call's stack footprint, matching the same static-buffer pattern
    // already used for s_rr_buf itself a few lines above. This function
    // is not reentrant and is only ever called from the single main
    // task, so static storage introduces no concurrency hazard here.
    static float candidates[RR_BUFFER_SIZE];
    size_t candidate_count = 0;
    for (size_t i = 0; i < s_rr_count; i++) {
        if (s_rr_buf[i].timestamp_us >= window_start_us) {
            candidates[candidate_count++] = s_rr_buf[i].rr_ms;
        }
    }

    // Rule 1: discard physiologically impossible intervals (outside
    // 300-2000ms, i.e. 30-200 BPM).
    static float stage1[RR_BUFFER_SIZE];
    size_t stage1_count = 0;
    for (size_t i = 0; i < candidate_count; i++) {
        if (candidates[i] >= 300.0f && candidates[i] <= 2000.0f) {
            stage1[stage1_count++] = candidates[i];
        }
    }

    if (stage1_count == 0) {
        // TEMPORARY DIAGNOSTIC — see the matching comment in
        // feed_ir_sample_for_peak_detection() above. candidate_count
        // tells us how many raw peaks the detector found in this
        // window before any filtering; stage1_count (always 0 here)
        // confirms none survived the 300-2000ms plausibility bound.
        // If candidate_count is also 0, the detector found no peaks at
        // all — a detection problem. If candidate_count is nonzero,
        // peaks were found but every single one fell outside
        // 300-2000ms — a filtering-threshold problem, not a detection
        // one, and a completely different fix.
        ESP_LOGW(TAG, "[HRV-DIAG] window has zero plausible RR intervals — "
                       "candidate_count=%u (raw peaks found before filtering)",
                  (unsigned)candidate_count);
        out->hrv_rmssd = out->hrv_mean_rr = out->hrv_sdnn = out->hrv_sd2 = -1.0f;
        return;
    }

    // Rule 2: discard intervals deviating more than 20% from the
    // window's median.
    static float sorted[RR_BUFFER_SIZE];
    memcpy(sorted, stage1, stage1_count * sizeof(float));
    qsort(sorted, stage1_count, sizeof(float), compare_floats);
    float median = sorted[stage1_count / 2];

    static float clean[RR_BUFFER_SIZE];
    size_t clean_count = 0;
    for (size_t i = 0; i < stage1_count; i++) {
        float deviation = fabsf(stage1[i] - median) / median;
        if (deviation <= 0.20f) {
            clean[clean_count++] = stage1[i];
        }
    }

    // Rule 3: require at least 30 clean intervals, else HRV is
    // "unavailable" for this window (-1.0f sentinel).
    if (clean_count < 30) {
        // TEMPORARY DIAGNOSTIC — see the two matching comments above.
        // This path means peaks were found AND survived the 300-2000ms
        // bound (stage1_count > 0), but too few survived the 20%-of-
        // median deviation check to reach the 30-interval minimum.
        ESP_LOGW(TAG, "[HRV-DIAG] too few clean RR intervals — "
                       "candidate_count=%u stage1_count=%u clean_count=%u (need >=30)",
                  (unsigned)candidate_count, (unsigned)stage1_count, (unsigned)clean_count);

        // TEMPORARY DIAGNOSTIC — Attempt 6. stage1 is already known-
        // plausible (300-2000ms) at this point but not yet median-
        // filtered, and it is still in the ORIGINAL order the peaks
        // were detected in — exactly what's needed to see whether the
        // ~4:1 spread seen this session (e.g. 399-1863ms within one
        // window) follows a short-long-short-long alternation. That
        // pattern would point at intermittent double-detection (a
        // spurious second peak on the same beat, producing one
        // artificially short interval) immediately followed by a
        // compensating artificially long one, or the reverse: a missed
        // beat producing one long interval that then measures short
        // against whatever comes next. Scattered, non-alternating
        // spread instead would point at genuine timing jitter in the
        // detector rather than a specific missed/double-detected beat
        // pattern, which would call for a different fix. Logged in
        // chunks of 10 to stay comfortably within a single safe log
        // line length even at the largest stage1_count seen so far
        // (82). Remove once the RR interval pattern is understood and
        // whatever fix it points to is confirmed.
        char line[128];
        for (size_t chunk_start = 0; chunk_start < stage1_count; chunk_start += 10) {
            int offset = snprintf(line, sizeof(line), "[HRV-DIAG-SEQ] idx%u-%u:",
                                    (unsigned)chunk_start,
                                    (unsigned)((chunk_start + 9 < stage1_count) ? chunk_start + 9 : stage1_count - 1));
            for (size_t i = chunk_start; i < stage1_count && i < chunk_start + 10; i++) {
                offset += snprintf(line + offset, sizeof(line) - offset, " %.0f", stage1[i]);
            }
            ESP_LOGW(TAG, "%s", line);
        }

        out->hrv_rmssd = out->hrv_mean_rr = out->hrv_sdnn = out->hrv_sd2 = -1.0f;
        return;
    }

    float mean_rr, sdnn;
    compute_mean_std(clean, clean_count, &mean_rr, &sdnn);

    float diff_sq_sum = 0;
    for (size_t i = 1; i < clean_count; i++) {
        float d = clean[i] - clean[i - 1];
        diff_sq_sum += d * d;
    }
    float rmssd = sqrtf(diff_sq_sum / (float)(clean_count - 1));

    // Poincaré SD1/SD2 — SD1 relates directly to RMSSD; SD2 derived from
    // it and SDNN, the standard relationship used throughout HRV
    // literature.
    float sd1 = rmssd / sqrtf(2.0f);
    float sd2_squared = 2.0f * sdnn * sdnn - sd1 * sd1;
    float sd2 = (sd2_squared > 0.0f) ? sqrtf(sd2_squared) : 0.0f;

    out->hrv_rmssd = rmssd;
    out->hrv_mean_rr = mean_rr;
    out->hrv_sdnn = sdnn;
    out->hrv_sd2 = sd2;
}

// --- The rest of state tracking. ---

static int s_tick_count = 0;
static feature_vector_t s_latest_features = {0};
static bool s_have_new_features = false;

static void feed_eda(void)
{
    // Same class of bug as compute_hrv_features() above, and arguably
    // higher-risk in practice since this function runs on every 250ms
    // tick rather than once per 5-second window: eda_sample_t is four
    // fields (~16 bytes with padding), so samples[150] alone reserved
    // ~2,400 bytes of this task's 3,584-byte stack budget on every
    // single call. Moved to static for the same reason.
    static eda_sample_t samples[150];
    size_t count = 0;
    eda_read_samples(samples, 150, &count);

    float sum = 0;
    size_t low_quality = 0;
    size_t uncalibrated_count = 0;
    size_t rejected_count = 0;
    for (size_t i = 0; i < count; i++) {
        sum += samples[i].conductance_us;
        // A reading counts against this tick's quality if it was
        // extrapolated beyond the 3-point calibration table, or if the
        // noise filter had to hold the last trusted value in its place
        // — either way it is not a genuine, in-range, fresh measurement
        // for this instant, and a window built mostly from these should
        // not be trusted as a calibration sample. See the doc comment
        // on s_eda_reject_frac_buf above and feed_calibration_window()
        // in ble_gatt_server.c, which is what actually acts on this.
        if (!samples[i].calibrated) uncalibrated_count++;
        if (samples[i].artifact_rejected) rejected_count++;
        if (!samples[i].calibrated || samples[i].artifact_rejected) low_quality++;
    }
    float avg = (count > 0) ? (sum / (float)count) : (s_eda_buf[(s_tick_count - 1 + WINDOW_TICKS) % WINDOW_TICKS]);
    float reject_frac = (count > 0) ? ((float)low_quality / (float)count) : 1.0f;

    s_eda_buf[s_tick_count % WINDOW_TICKS] = avg;
    s_eda_reject_frac_buf[s_tick_count % WINDOW_TICKS] = reject_frac;

    // Populate the batch summary main.c's console line needs — this is
    // the ONLY read of the EDA sensor for this tick; main.c no longer
    // calls eda_read_samples() itself. See the doc comment above
    // s_last_eda_summary's declaration for why.
    if (count > 0) {
        eda_sample_t latest = samples[count - 1];
        s_last_eda_summary = (eda_batch_summary_t){
            .count = count,
            .latest_voltage = latest.voltage,
            .latest_resistance_ohms = latest.resistance_ohms,
            .latest_conductance_us = latest.conductance_us,
            .latest_calibrated = latest.calibrated,
            .latest_artifact_rejected = latest.artifact_rejected,
            .uncalibrated_count = uncalibrated_count,
            .rejected_count = rejected_count,
        };
    } else {
        s_last_eda_summary = (eda_batch_summary_t){0};
    }
}

static void feed_temp(void)
{
    float temp_c = 0.0f;
    if (!ds18b20_get_temp_4hz(&temp_c)) {
        // No reading yet (only happens for the first ~1s at boot) —
        // hold at 0 rather than leave uninitialised memory.
        temp_c = (s_tick_count == 0) ? 0.0f : s_temp_buf[(s_tick_count - 1 + WINDOW_TICKS) % WINDOW_TICKS];
    }
    s_temp_buf[s_tick_count % WINDOW_TICKS] = temp_c;
}

static void feed_accel(void)
{
    // Same static-buffer fix as feed_eda() above and for the same
    // reason: this runs every 250ms tick, and a 150-element sample
    // array on the stack was consuming a large, avoidable share of a
    // 3,584-byte task stack on every call.
    static mpu6500_sample_t samples[150];
    size_t count = 0;
    mpu6500_read_samples(samples, 150, &count);

    // Populate the batch summary main.c's console line needs, computed
    // over the RAW samples before any of the downsampling/grouping
    // below — this is the ONLY read of the accelerometer for this
    // tick; main.c no longer calls mpu6500_read_samples() itself. See
    // the doc comment above s_last_accel_summary's declaration for why.
    if (count > 0) {
        float mag_sum = 0.0f, mag_min = 0.0f, mag_max = 0.0f;
        for (size_t i = 0; i < count; i++) {
            mpu6500_sample_t s = samples[i];
            float mag = sqrtf(s.x_g * s.x_g + s.y_g * s.y_g + s.z_g * s.z_g);
            if (i == 0) { mag_min = mag; mag_max = mag; }
            if (mag < mag_min) mag_min = mag;
            if (mag > mag_max) mag_max = mag;
            mag_sum += mag;
        }
        s_last_accel_summary = (accel_batch_summary_t){
            .count = count,
            .avg_magnitude_g = mag_sum / (float)count,
            .min_magnitude_g = mag_min,
            .max_magnitude_g = mag_max,
        };
    } else {
        s_last_accel_summary = (accel_batch_summary_t){0};
    }

    // SIMPLIFICATION, FLAGGED: proper downsampling from ~100Hz to 32Hz
    // would apply a low-pass filter before picking points, to avoid
    // aliasing. This splits each tick's batch into 8 roughly-equal
    // groups and averages each group instead — a reasonable first pass,
    // not a substitute for real anti-alias filtering. Revisit if
    // acc_mag features look unexpectedly noisy once compared against
    // real captured motion data.
    //
    // BUG FIX: this function used to `break` out of the group loop the
    // moment `start >= count`, which meant that whenever
    // mpu6500_read_samples() returned fewer than ACC_SAMPLES_PER_TICK
    // (8) usable samples for this tick — plausible given this sensor
    // shares an I2C bus with the MAX30102 and per-tick timing between
    // the two is not perfectly synchronised — s_acc_write_index
    // advanced by fewer than 8 positions for that tick. That shortfall
    // was persistent, not self-correcting: over enough ticks it left
    // large stretches of the 1920-slot window buffer permanently at
    // their zero-initialised default, while the ticks that did write
    // held genuine ~0.95g values. Real hardware showed the resulting
    // window statistics exactly matching that signature — a mean of
    // 0.316g and a standard deviation of 0.450g in a window whose every
    // individual tick printed a consistent ~0.958g — which also forced
    // acc_activity to 1 on every window regardless of actual movement,
    // independently blocking calibration.
    //
    // The fix holds the last real group average forward to fill out
    // any remaining groups for this tick, the same hold-last pattern
    // this codebase already uses for temperature upsampling in
    // feed_temp() above, guaranteeing the write index always advances
    // by exactly ACC_SAMPLES_PER_TICK regardless of how few raw samples
    // were actually available.
    if (count == 0) {
        // No new samples at all this tick — hold the last written
        // group value across all 8 slots rather than leaving them
        // unwritten.
        size_t last_index = (s_acc_write_index + ACC_WINDOW_SIZE - 1) % ACC_WINDOW_SIZE;
        float last_value = s_acc_mag_buf[last_index];
        for (int g = 0; g < ACC_SAMPLES_PER_TICK; g++) {
            s_acc_mag_buf[s_acc_write_index] = last_value;
            s_acc_write_index = (s_acc_write_index + 1) % ACC_WINDOW_SIZE;
        }
        return;
    }

    size_t per_group = count / ACC_SAMPLES_PER_TICK;
    if (per_group == 0) per_group = 1;

    float last_group_value = s_acc_mag_buf[(s_acc_write_index + ACC_WINDOW_SIZE - 1) % ACC_WINDOW_SIZE];
    for (int g = 0; g < ACC_SAMPLES_PER_TICK; g++) {
        size_t start = g * per_group;
        size_t end = (g == ACC_SAMPLES_PER_TICK - 1) ? count : (start + per_group);
        if (start >= count) {
            // Ran out of real samples for this tick's remaining groups
            // — hold the last real group's value instead of leaving
            // this slot, and every slot after it, unwritten.
            s_acc_mag_buf[s_acc_write_index] = last_group_value;
            s_acc_write_index = (s_acc_write_index + 1) % ACC_WINDOW_SIZE;
            continue;
        }
        if (end > count) end = count;

        float sum_mag = 0;
        size_t n = 0;
        for (size_t i = start; i < end; i++) {
            mpu6500_sample_t s = samples[i];
            sum_mag += sqrtf(s.x_g * s.x_g + s.y_g * s.y_g + s.z_g * s.z_g);
            n++;
        }
        if (n > 0) {
            last_group_value = sum_mag / (float)n;
            s_acc_mag_buf[s_acc_write_index] = last_group_value;
            s_acc_write_index = (s_acc_write_index + 1) % ACC_WINDOW_SIZE;
        }
    }
}

// TEMPORARY DIAGNOSTIC — Attempt 5. Two separate real-hardware
// captures, on two separate sessions, both showed the baseline gap
// (ac_value) decaying with a real-world time constant of roughly
// 53-58 seconds, against a coded time constant of 5 seconds implied by
// BASELINE_EMA_ALPHA=0.002 at an assumed ~100Hz feed rate — a
// consistent ~10-11x discrepancy across independent runs, which rules
// out a one-off settling artifact and points at something structural.
// One candidate explanation: if max30102_read_ir_samples() is
// returning batches where only a fraction of the "count" reported
// values are genuinely new distinct readings, with the remainder being
// repeats of an already-seen value, then s_ir_baseline_ema would be
// re-processing the same value multiple times per real update, which
// would produce exactly this kind of ~10x-slower-than-coded effective
// time constant without any bug in the EMA math itself. This also
// offers a candidate explanation for the suspiciously regular ~5-5.5s
// peak-to-peak spacing logged by [HRV-DIAG] in both captures so far —
// a staircase-shaped input (hold, then jump) crossing a threshold on
// each jump would produce exactly this kind of artificially even
// spacing, as opposed to the ~0.6-1.0s irregular spacing a genuine
// pulse produces. This log makes that testable directly: it prints the
// first 5 raw values of one batch, once roughly every 2 seconds (every
// 8th tick), plus a same-batch duplicate count across all samples in
// that batch. Remove once resolved.
#define HR_BATCH_DIAG_LOG_EVERY_N_TICKS 8
static int s_hr_batch_diag_tick_counter = 0;

static void feed_heart_rate(void)
{
    // Same static-buffer fix as the three functions above.
    static uint32_t samples[150];
    size_t count = 0;
    max30102_read_ir_samples(samples, 150, &count);

    // Populate the batch summary main.c's console line needs — this is
    // the ONLY read of the heart-rate sensor for this tick; main.c no
    // longer calls max30102_read_ir_samples() itself. This is the exact
    // fix for the starvation this session's [HRV-DIAG-BATCH] logging
    // confirmed: main.c's own former call to this same function, for
    // its console line, was draining most of each tick's samples
    // before this line ever ran. See the doc comment above
    // s_last_hr_summary's declaration for the full history.
    if (count > 0) {
        uint32_t min_val = samples[0], max_val = samples[0];
        for (size_t i = 1; i < count; i++) {
            if (samples[i] < min_val) min_val = samples[i];
            if (samples[i] > max_val) max_val = samples[i];
        }
        s_last_hr_summary = (hr_batch_summary_t){
            .count = count,
            .latest = samples[count - 1],
            .min = min_val,
            .max = max_val,
        };
    } else {
        s_last_hr_summary = (hr_batch_summary_t){0};
    }

    // TEMPORARY DIAGNOSTIC — Attempt 5, see the doc comment above
    // s_hr_batch_diag_tick_counter's declaration.
    s_hr_batch_diag_tick_counter++;
    if (s_hr_batch_diag_tick_counter >= HR_BATCH_DIAG_LOG_EVERY_N_TICKS) {
        s_hr_batch_diag_tick_counter = 0;
        size_t dup_count = 0;
        for (size_t i = 1; i < count; i++) {
            if (samples[i] == samples[i - 1]) dup_count++;
        }
        ESP_LOGI(TAG, "[HRV-DIAG-BATCH] count=%u first5=[%u,%u,%u,%u,%u] "
                       "consecutive_duplicates=%u/%u",
                  (unsigned)count,
                  (unsigned)(count > 0 ? samples[0] : 0),
                  (unsigned)(count > 1 ? samples[1] : 0),
                  (unsigned)(count > 2 ? samples[2] : 0),
                  (unsigned)(count > 3 ? samples[3] : 0),
                  (unsigned)(count > 4 ? samples[4] : 0),
                  (unsigned)dup_count, (unsigned)(count > 0 ? count - 1 : 0));
    }

    // Peak detection runs on the RAW ~100Hz stream, not a 64Hz-
    // decimated version. DEVIATION FROM chat2_firmware_handoff.md,
    // FLAGGED: that document specifies downsampling BVP to 64Hz before
    // feature extraction. For RR-interval timing specifically, using
    // the raw acquisition rate gives more accurate peak timestamps —
    // decimating first would only throw away timing precision that
    // directly affects HRV accuracy. Every other feature group follows
    // the locked downsampling contract; this is a deliberate, narrow
    // exception for HRV timing only.
    //
    // Approximate real timestamp per sample, spreading this tick's
    // batch evenly across the 250ms tick interval.
    int64_t tick_end_us = esp_timer_get_time();
    int64_t tick_start_us = tick_end_us - (TICK_MS * 1000);
    for (size_t i = 0; i < count; i++) {
        int64_t ts = tick_start_us + (int64_t)((float)i / (float)count * (TICK_MS * 1000));
        feed_ir_sample_for_peak_detection(samples[i], ts);
    }
}

esp_err_t feature_extraction_init(void)
{
    memset(s_eda_buf, 0, sizeof(s_eda_buf));
    memset(s_temp_buf, 0, sizeof(s_temp_buf));
    memset(s_acc_mag_buf, 0, sizeof(s_acc_mag_buf));
    ESP_LOGI(TAG, "Feature extraction ready — 60s window, 5s step");
    return ESP_OK;
}

void feature_extraction_tick(void)
{
    feed_eda();
    feed_temp();
    feed_accel();
    feed_heart_rate();

    s_tick_count++;

    if (s_tick_count < WINDOW_TICKS) {
        return;  // not a full 60s of data yet
    }
    if (s_tick_count % STEP_TICKS != 0) {
        return;  // not a 5s step boundary
    }

    feature_vector_t fv = {0};

    compute_mean_std(s_eda_buf, WINDOW_TICKS, &fv.eda_mean, &fv.eda_std);
    fv.eda_slope = compute_slope(s_eda_buf, WINDOW_TICKS);
    float eda_min = s_eda_buf[0], eda_max = s_eda_buf[0];
    for (int i = 1; i < WINDOW_TICKS; i++) {
        if (s_eda_buf[i] < eda_min) eda_min = s_eda_buf[i];
        if (s_eda_buf[i] > eda_max) eda_max = s_eda_buf[i];
    }
    fv.eda_range = eda_max - eda_min;

    // temp_mean and temp_std both come from the same helper.
    float temp_std_unused;
    compute_mean_std(s_temp_buf, WINDOW_TICKS, &fv.temp_mean, &temp_std_unused);
    fv.temp_slope = compute_slope(s_temp_buf, WINDOW_TICKS);

    compute_mean_std(s_acc_mag_buf, ACC_WINDOW_SIZE, &fv.acc_mag_mean, &fv.acc_mag_std);
    fv.acc_activity = (fv.acc_mag_std > 0.02f) ? 1.0f : 0.0f;

    // Average this window's per-tick EDA reject fraction (see
    // s_eda_reject_frac_buf's doc comment and feed_eda() above). Only
    // the mean is needed here, so the std output is discarded, same
    // pattern already used for temp_std_unused above.
    float eda_reject_frac_std_unused;
    compute_mean_std(s_eda_reject_frac_buf, WINDOW_TICKS, &fv.eda_reject_frac, &eda_reject_frac_std_unused);

    int64_t window_start_us = esp_timer_get_time() - (int64_t)WINDOW_TICKS * TICK_MS * 1000;
    compute_hrv_features(window_start_us, &fv);

    s_latest_features = fv;
    s_have_new_features = true;

    ESP_LOGI(TAG, "New feature window ready (tick %d)", s_tick_count);
}

bool feature_extraction_get_latest(feature_vector_t *out)
{
    if (!s_have_new_features) return false;
    *out = s_latest_features;
    s_have_new_features = false;
    return true;
}