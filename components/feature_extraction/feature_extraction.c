#include "feature_extraction.h"

#include <string.h>
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

static float s_ir_prev_sample = 0.0f;
static bool s_ir_rising = false;
static int s_refractory_countdown = 0;
static int64_t s_last_peak_time_us = 0;
static bool s_have_last_peak = false;

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

    bool now_rising = (ac_value > s_ir_prev_sample);

    if (s_refractory_countdown > 0) {
        s_refractory_countdown--;
    } else if (s_ir_rising && !now_rising && s_ir_prev_sample > threshold) {
        // Signal just turned from rising to falling, above threshold —
        // this is a peak.
        if (s_have_last_peak) {
            float rr_ms = (float)(timestamp_us - s_last_peak_time_us) / 1000.0f;
            s_rr_buf[s_rr_write_index] = (rr_entry_t){ .rr_ms = rr_ms, .timestamp_us = timestamp_us };
            s_rr_write_index = (s_rr_write_index + 1) % RR_BUFFER_SIZE;
            if (s_rr_count < RR_BUFFER_SIZE) s_rr_count++;
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
            ESP_LOGI(TAG, "[HRV-DIAG] peak detected, raw rr_ms=%.1f (threshold=%.1f, ac_value=%.1f)",
                      rr_ms, threshold, ac_value);
        }
        s_last_peak_time_us = timestamp_us;
        s_have_last_peak = true;
        s_refractory_countdown = PEAK_REFRACTORY_SAMPLES;
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
        if (!samples[i].calibrated || samples[i].artifact_rejected) low_quality++;
    }
    float avg = (count > 0) ? (sum / (float)count) : (s_eda_buf[(s_tick_count - 1 + WINDOW_TICKS) % WINDOW_TICKS]);
    float reject_frac = (count > 0) ? ((float)low_quality / (float)count) : 1.0f;

    s_eda_buf[s_tick_count % WINDOW_TICKS] = avg;
    s_eda_reject_frac_buf[s_tick_count % WINDOW_TICKS] = reject_frac;
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

static void feed_heart_rate(void)
{
    // Same static-buffer fix as the three functions above.
    static uint32_t samples[150];
    size_t count = 0;
    max30102_read_ir_samples(samples, 150, &count);

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