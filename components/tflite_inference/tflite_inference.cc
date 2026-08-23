#include "tflite_inference.h"

#include <cstring>
#include <cmath>
#include "esp_log.h"

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "stress_nn_data.h"
#include "validation_set.h"

static const char *TAG = "tflite_inference";

// --- Quantization constants, locked in chat2_firmware_handoff.md,
//     confirmed against the real ml_pipeline_documentation.md export
//     (100% match rate between the float and int8 models). ---
#define QUANT_SCALE_IN   0.037689f
#define QUANT_ZP_IN      (-19)
#define QUANT_SCALE_OUT  0.003906f
#define QUANT_ZP_OUT     (-128)
#define THRESHOLD_WATCH  0.40f
#define THRESHOLD_STRESS 0.70f

// Starting guess, per chat2_firmware_handoff.md's own open question —
// the model itself is 4,792 bytes and this is a tiny 13->32->16->1
// network, so 20KB is a generous first try, but this has not yet been
// empirically confirmed on real hardware. Watch the init log below: if
// AllocateTensors() fails, this needs to grow.
constexpr int kTensorArenaSize = 20 * 1024;
static uint8_t s_tensor_arena[kTensorArenaSize];

static const tflite::Model *s_model = nullptr;
static tflite::MicroInterpreter *s_interpreter = nullptr;
static TfLiteTensor *s_input = nullptr;
static TfLiteTensor *s_output = nullptr;
static bool s_ready = false;

// TEMPORARY passthrough calibration — see tflite_inference.h for why.
static float s_calibration_mean[13] = {0};
static float s_calibration_std[13]  = {1,1,1,1,1,1,1,1,1,1,1,1,1};
static bool s_calibration_set = false;

// --- Shared quantize -> invoke -> dequantize path, used both by real
//     inference (tflite_inference_run) and by the boot-time validation
//     check below. Takes ALREADY-NORMALISED (z-scored) features — the
//     validation set from ml_pipeline_documentation.md is stored
//     pre-normalised, and real inference normalises before calling
//     this, so this one path serves both correctly. ---
static bool quantize_invoke_dequantize(const float z_scored[13], float *out_probability)
{
    if (!s_ready) return false;

    for (int i = 0; i < 13; i++) {
        int32_t q = (int32_t)lroundf(z_scored[i] / QUANT_SCALE_IN) + QUANT_ZP_IN;
        if (q < -128) q = -128;
        if (q > 127) q = 127;
        s_input->data.int8[i] = (int8_t)q;
    }

    if (s_interpreter->Invoke() != kTfLiteOk) {
        ESP_LOGW(TAG, "Invoke() failed");
        return false;
    }

    int8_t out_q = s_output->data.int8[0];
    *out_probability = ((float)out_q - QUANT_ZP_OUT) * QUANT_SCALE_OUT;
    return true;
}

// --- Boot-time validation check. ---
//
// PLAIN-LANGUAGE SUMMARY:
// Runs the 50 known feature vectors from validation_set.h through the
// model right here on this board, and checks that at least 96% of the
// predictions match the expected answer — exactly the on-device check
// ml_pipeline_documentation.md describes. This catches a bad build or a
// quantization mismatch immediately at boot, instead of only being
// discovered later during real monitoring.
//
// JUDGMENT CALL, FLAGGED: validation_labels are binary (0/1), not the
// three-tier 0/1/2 used for real monitoring. For this specific check,
// a prediction counts as matching if (probability >= 0.5) agrees with
// the stored label — a direct binary comparison. This wasn't spelled
// out in the handoff documents, so this is my own reasonable reading
// of "compare against ground truth," not a locked project decision.
static void run_validation_check(void)
{
    int correct = 0;
    for (int i = 0; i < VALIDATION_N; i++) {
        float probability = 0.0f;
        if (!quantize_invoke_dequantize(validation_features[i], &probability)) {
            ESP_LOGE(TAG, "Validation vector %d: inference call failed", i);
            continue;
        }
        int predicted = (probability >= 0.5f) ? 1 : 0;
        if (predicted == validation_labels[i]) {
            correct++;
        }
    }

    float match_rate = (float)correct / (float)VALIDATION_N * 100.0f;
    if (match_rate >= 96.0f) {
        ESP_LOGI(TAG, "On-device validation PASSED: %d/%d (%.1f%%)", correct, VALIDATION_N, match_rate);
    } else {
        ESP_LOGE(TAG, "On-device validation FAILED: only %d/%d (%.1f%%) matched — expected >= 96%%. "
                       "Something is wrong: a bad build, wrong quantization constants, or a corrupted "
                       "model file. Do not trust real inference results until this is investigated.",
                  correct, VALIDATION_N, match_rate);
    }
}

esp_err_t tflite_inference_init(void)
{
    s_model = tflite::GetModel(_kaggle_working_stress_nn_tflite);
    if (s_model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "Model schema version mismatch: model is v%lu, this build of "
                       "esp-tflite-micro expects v%d", s_model->version(), TFLITE_SCHEMA_VERSION);
        return ESP_FAIL;
    }

    // The trained architecture (ml_pipeline_documentation.md):
    // 13 -> Dense(32, ReLU) -> Dense(16, ReLU) -> Dense(1, Sigmoid).
    // ReLU is fused into FULLY_CONNECTED's own activation, so only two
    // distinct op kinds actually need registering here.
    static tflite::MicroMutableOpResolver<2> resolver;
    resolver.AddFullyConnected();
    resolver.AddLogistic();

    static tflite::MicroInterpreter static_interpreter(s_model, resolver, s_tensor_arena, kTensorArenaSize);
    s_interpreter = &static_interpreter;

    TfLiteStatus alloc_status = s_interpreter->AllocateTensors();
    if (alloc_status != kTfLiteOk) {
        ESP_LOGE(TAG, "AllocateTensors() failed — the %dKB tensor arena is too small for this "
                       "model. Increase kTensorArenaSize in tflite_inference.cc and try again.",
                  kTensorArenaSize / 1024);
        return ESP_FAIL;
    }

    s_input = s_interpreter->input(0);
    s_output = s_interpreter->output(0);
    s_ready = true;

    ESP_LOGI(TAG, "Model loaded (%u bytes). Tensor arena used: %u / %d bytes",
              (unsigned)_kaggle_working_stress_nn_tflite_len,
              (unsigned)s_interpreter->arena_used_bytes(), kTensorArenaSize);

    run_validation_check();

    if (!s_calibration_set) {
        ESP_LOGW(TAG, "No real calibration data set yet — inference will run UN-normalised "
                       "(mean=0, std=1 passthrough) until tflite_inference_set_calibration() is "
                       "called with real per-user values from the BLE calibration flow. Real "
                       "inference results are not meaningful until then.");
    }

    return ESP_OK;
}

void tflite_inference_set_calibration(const float mean[13], const float std[13])
{
    memcpy(s_calibration_mean, mean, sizeof(s_calibration_mean));
    memcpy(s_calibration_std, std, sizeof(s_calibration_std));
    s_calibration_set = true;
    ESP_LOGI(TAG, "Calibration data set — inference now normalises against real per-user values");
}

bool tflite_inference_run(const float features[13], int *out_class, float *out_probability)
{
    if (!s_ready) return false;

    float z_scored[13];
    for (int i = 0; i < 13; i++) {
        float std_dev = s_calibration_std[i];
        z_scored[i] = (fabsf(std_dev) > 1e-6f) ? (features[i] - s_calibration_mean[i]) / std_dev
                                                : features[i];
    }

    float probability = 0.0f;
    if (!quantize_invoke_dequantize(z_scored, &probability)) {
        return false;
    }

    int cls;
    if (probability < THRESHOLD_WATCH) {
        cls = 0;
    } else if (probability < THRESHOLD_STRESS) {
        cls = 1;
    } else {
        cls = 2;
    }

    *out_class = cls;
    *out_probability = probability;
    return true;
}
