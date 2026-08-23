#include "ble_gatt_server.h"

#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "ble_gatt_server";

// =====================================================================
// UUIDs — computed directly from the strings locked in
// ble_gatt_contract_handoff.md. NimBLE stores 128-bit UUIDs
// byte-reversed from how they're normally written, so these were
// converted carefully rather than typed by hand in the "wrong" order.
// =====================================================================
static const ble_uuid128_t SVC_UUID =
    BLE_UUID128_INIT(0x2f,0x1e,0x0d,0x9c,0x8b,0x7a,0x6f,0x5e,0x4d,0x3c,0x2b,0x1a,0x00,0x00,0xb3,0xa5);
static const ble_uuid128_t CHR_DEVICE_STATE_UUID =
    BLE_UUID128_INIT(0x2f,0x1e,0x0d,0x9c,0x8b,0x7a,0x6f,0x5e,0x4d,0x3c,0x2b,0x1a,0x01,0x00,0xb3,0xa5);
static const ble_uuid128_t CHR_CONTROL_COMMAND_UUID =
    BLE_UUID128_INIT(0x2f,0x1e,0x0d,0x9c,0x8b,0x7a,0x6f,0x5e,0x4d,0x3c,0x2b,0x1a,0x02,0x00,0xb3,0xa5);
static const ble_uuid128_t CHR_INFERENCE_OUTPUT_UUID =
    BLE_UUID128_INIT(0x2f,0x1e,0x0d,0x9c,0x8b,0x7a,0x6f,0x5e,0x4d,0x3c,0x2b,0x1a,0x03,0x00,0xb3,0xa5);
static const ble_uuid128_t CHR_FEATURE_VECTOR_UUID =
    BLE_UUID128_INIT(0x2f,0x1e,0x0d,0x9c,0x8b,0x7a,0x6f,0x5e,0x4d,0x3c,0x2b,0x1a,0x04,0x00,0xb3,0xa5);
static const ble_uuid128_t CHR_CALIBRATION_STATS_UUID =
    BLE_UUID128_INIT(0x2f,0x1e,0x0d,0x9c,0x8b,0x7a,0x6f,0x5e,0x4d,0x3c,0x2b,0x1a,0x05,0x00,0xb3,0xa5);

// Battery service — standard Bluetooth SIG-assigned UUIDs, not custom.
#define BATTERY_SVC_UUID       0x180F
#define BATTERY_LEVEL_CHR_UUID 0x2A19

// =====================================================================
// Device state machine
// =====================================================================
typedef enum {
    STATE_UNCALIBRATED = 0x00,
    STATE_IDLE          = 0x01,
    STATE_CALIBRATING   = 0x02,
    STATE_MONITORING    = 0x03,
    STATE_NOT_WORN      = 0x04,
    STATE_ERROR         = 0x05,
    STATE_PENDING_RESET = 0x06,
} device_state_t;

typedef enum {
    ERR_I2C_BUS_FAILURE       = 0x01,
    ERR_ONE_WIRE_FAILURE      = 0x02,
    ERR_EDA_ADC_FAILURE       = 0x03,
    ERR_TFLITE_ALLOC_FAILURE  = 0x04,
    ERR_NVS_FAILURE           = 0x05,
    ERR_PERSISTENT_SIGNAL     = 0x06,
} error_code_t;

static device_state_t s_state = STATE_UNCALIBRATED;
static uint8_t s_aux_byte = 0;  // calibration progress 0-100, or error code
static device_state_t s_pre_reset_state = STATE_UNCALIBRATED;
static esp_timer_handle_t s_reset_timer = NULL;

// =====================================================================
// Connection + subscription tracking
// =====================================================================
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_val_device_state, s_val_control_command, s_val_inference_output;
static uint16_t s_val_feature_vector, s_val_calibration_stats, s_val_battery_level;
static bool s_sub_device_state = false;
static bool s_sub_inference_output = false;
static bool s_sub_feature_vector = false;
static bool s_sub_battery_level = false;

// =====================================================================
// Calibration accumulator (per-feature running mean/std, per
// ble_gatt_contract_handoff.md: >= 24 valid windows within 3 minutes,
// windows with acc_activity=1 excluded from statistics and the count)
// =====================================================================
#define CALIBRATION_MIN_VALID_WINDOWS 24
#define CALIBRATION_MAX_WINDOWS       36  // 3 minutes / 5s step
static double s_cal_sum[13];
static double s_cal_sum_sq[13];
static int s_cal_valid_count = 0;
static int s_cal_windows_seen = 0;
static float s_cal_mean[13];
static float s_cal_std[13];

// =====================================================================
// Wear detection (temperature-primary, 2.0C hysteresis, per the
// locked contract)
// =====================================================================
#define WEAR_ENTER_NOT_WORN_C 28.0f
#define WEAR_RETURN_MONITOR_C 30.0f

static nvs_handle_t s_nvs_handle;

// Forward declarations
static void set_state(device_state_t new_state, uint8_t aux);
static void notify_device_state(void);
static int gatt_svr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);
static void reset_timeout_cb(void *arg);

// =====================================================================
// Feature vector -> 13-float array, matching feature_extraction.h's
// locked field order.
// =====================================================================
static void feature_vector_to_array(const feature_vector_t *fv, float out[13])
{
    out[0] = fv->eda_mean;      out[1] = fv->eda_std;
    out[2] = fv->eda_slope;     out[3] = fv->eda_range;
    out[4] = fv->hrv_rmssd;     out[5] = fv->hrv_mean_rr;
    out[6] = fv->hrv_sdnn;      out[7] = fv->hrv_sd2;
    out[8] = fv->temp_mean;     out[9] = fv->temp_slope;
    out[10] = fv->acc_mag_mean; out[11] = fv->acc_mag_std;
    out[12] = fv->acc_activity;
}

// =====================================================================
// NVS: calibration persistence
// =====================================================================
static void load_calibration_from_nvs(void)
{
    size_t required_size = sizeof(s_cal_mean);
    esp_err_t err = nvs_get_blob(s_nvs_handle, "cal_mean", s_cal_mean, &required_size);
    if (err != ESP_OK) return;
    required_size = sizeof(s_cal_std);
    err = nvs_get_blob(s_nvs_handle, "cal_std", s_cal_std, &required_size);
    if (err != ESP_OK) return;

    uint8_t calibrated = 0;
    required_size = sizeof(calibrated);
    nvs_get_u8(s_nvs_handle, "calibrated", &calibrated);
    if (calibrated) {
        s_state = STATE_IDLE;
        ESP_LOGI(TAG, "Loaded existing calibration from NVS — starting in IDLE");
    }
}

static bool save_calibration_to_nvs(void)
{
    esp_err_t err = nvs_set_blob(s_nvs_handle, "cal_mean", s_cal_mean, sizeof(s_cal_mean));
    if (err != ESP_OK) { ESP_LOGE(TAG, "NVS write (mean) failed: %s", esp_err_to_name(err)); return false; }
    err = nvs_set_blob(s_nvs_handle, "cal_std", s_cal_std, sizeof(s_cal_std));
    if (err != ESP_OK) { ESP_LOGE(TAG, "NVS write (std) failed: %s", esp_err_to_name(err)); return false; }
    err = nvs_set_u8(s_nvs_handle, "calibrated", 1);
    if (err != ESP_OK) { ESP_LOGE(TAG, "NVS write (flag) failed: %s", esp_err_to_name(err)); return false; }
    nvs_commit(s_nvs_handle);
    return true;
}

// =====================================================================
// Calibration accumulation — called from ble_gatt_server_notify_features()
// whenever the device is CALIBRATING.
// =====================================================================
static void feed_calibration_window(const feature_vector_t *fv)
{
    s_cal_windows_seen++;

    float arr[13];
    feature_vector_to_array(fv, arr);

    // A valid window: all 13 features computed without NaN/sentinel
    // substitution, AND acc_activity == 0 (per the locked contract).
    bool valid = (fv->acc_activity == 0.0f) &&
                 (fv->hrv_rmssd != -1.0f) && (fv->hrv_mean_rr != -1.0f) &&
                 (fv->hrv_sdnn != -1.0f) && (fv->hrv_sd2 != -1.0f);

    if (valid) {
        for (int i = 0; i < 13; i++) {
            s_cal_sum[i] += arr[i];
            s_cal_sum_sq[i] += (double)arr[i] * (double)arr[i];
        }
        s_cal_valid_count++;
    }

    uint8_t progress = (uint8_t)((s_cal_valid_count * 100) / CALIBRATION_MIN_VALID_WINDOWS);
    if (progress > 100) progress = 100;
    s_aux_byte = progress;
    notify_device_state();

    if (s_cal_valid_count >= CALIBRATION_MIN_VALID_WINDOWS) {
        for (int i = 0; i < 13; i++) {
            double n = s_cal_valid_count;
            double mean = s_cal_sum[i] / n;
            double variance = (s_cal_sum_sq[i] / n) - (mean * mean);
            if (variance < 0) variance = 0;  // floating-point guard
            s_cal_mean[i] = (float)mean;
            s_cal_std[i] = (float)sqrt(variance);
        }
        if (save_calibration_to_nvs()) {
            ESP_LOGI(TAG, "Calibration complete: %d valid windows", s_cal_valid_count);
            set_state(STATE_IDLE, 0);
        } else {
            set_state(STATE_ERROR, ERR_NVS_FAILURE);
        }
        return;
    }

    if (s_cal_windows_seen >= CALIBRATION_MAX_WINDOWS) {
        ESP_LOGW(TAG, "Calibration timed out: only %d/%d valid windows in %d windows seen",
                  s_cal_valid_count, CALIBRATION_MIN_VALID_WINDOWS, s_cal_windows_seen);
        set_state(STATE_UNCALIBRATED, 0);
    }
}

// =====================================================================
// State transitions
// =====================================================================
static void set_state(device_state_t new_state, uint8_t aux)
{
    s_state = new_state;
    s_aux_byte = aux;
    ESP_LOGI(TAG, "State -> 0x%02X (aux=0x%02X)", s_state, s_aux_byte);
    notify_device_state();
}

static void reset_timeout_cb(void *arg)
{
    if (s_state == STATE_PENDING_RESET) {
        ESP_LOGI(TAG, "Reset confirmation timed out — restoring previous state");
        set_state(s_pre_reset_state, 0);
    }
}

// =====================================================================
// Control Command dispatch — validity per the locked command table.
// =====================================================================
static int handle_control_command(uint8_t cmd)
{
    switch (cmd) {
        case 0x01:  // startCalibration
            if (s_state != STATE_UNCALIBRATED && s_state != STATE_IDLE) return -1;
            memset(s_cal_sum, 0, sizeof(s_cal_sum));
            memset(s_cal_sum_sq, 0, sizeof(s_cal_sum_sq));
            s_cal_valid_count = 0;
            s_cal_windows_seen = 0;
            set_state(STATE_CALIBRATING, 0);
            return 0;

        case 0x02:  // startIntervention
            if (s_state != STATE_IDLE) return -1;
            nvs_set_u8(s_nvs_handle, "should_monitor", 1);
            nvs_commit(s_nvs_handle);
            set_state(STATE_MONITORING, 0);
            return 0;

        case 0x03:  // stopIntervention
            if (s_state != STATE_MONITORING && s_state != STATE_NOT_WORN) return -1;
            nvs_set_u8(s_nvs_handle, "should_monitor", 0);
            nvs_commit(s_nvs_handle);
            set_state(STATE_IDLE, 0);
            return 0;

        case 0x04:  // requestReset
            s_pre_reset_state = s_state;
            set_state(STATE_PENDING_RESET, 0);
            esp_timer_start_once(s_reset_timer, 15000000);  // 15s, in microseconds
            return 0;

        case 0x05:  // confirmReset
            if (s_state != STATE_PENDING_RESET) return -1;
            esp_timer_stop(s_reset_timer);
            nvs_erase_key(s_nvs_handle, "cal_mean");
            nvs_erase_key(s_nvs_handle, "cal_std");
            nvs_set_u8(s_nvs_handle, "calibrated", 0);
            nvs_set_u8(s_nvs_handle, "should_monitor", 0);
            nvs_commit(s_nvs_handle);
            set_state(STATE_UNCALIBRATED, 0);
            return 0;

        default:
            return -1;
    }
}

// =====================================================================
// Notification builders
// =====================================================================
static void notify_device_state(void)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_sub_device_state) return;

    uint8_t payload[2];
    int len;
    if (s_state == STATE_CALIBRATING || s_state == STATE_ERROR) {
        payload[0] = (uint8_t)s_state;
        payload[1] = s_aux_byte;
        len = 2;
    } else {
        payload[0] = (uint8_t)s_state;
        len = 1;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(payload, len);
    ble_gatts_notify_custom(s_conn_handle, s_val_device_state, om);
}

void ble_gatt_server_notify_inference(int stress_class, float probability, uint8_t window_sequence)
{
    if (s_state != STATE_MONITORING) return;
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_sub_inference_output) return;

    uint8_t payload[3];
    payload[0] = (uint8_t)stress_class;
    float clamped = probability;
    if (clamped < 0.0f) clamped = 0.0f;
    if (clamped > 1.0f) clamped = 1.0f;
    payload[1] = (uint8_t)(clamped * 255.0f);
    payload[2] = window_sequence;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(payload, sizeof(payload));
    ble_gatts_notify_custom(s_conn_handle, s_val_inference_output, om);
}

void ble_gatt_server_notify_features(const feature_vector_t *features, uint8_t window_sequence)
{
    if (s_state == STATE_CALIBRATING) {
        feed_calibration_window(features);
    }

    if (s_state != STATE_MONITORING && s_state != STATE_CALIBRATING) return;
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_sub_feature_vector) return;

    uint8_t payload[53];
    payload[0] = window_sequence;

    float arr[13];
    feature_vector_to_array(features, arr);
    memcpy(&payload[1], arr, sizeof(arr));  // 52 bytes, float32 little-endian
                                             // (ESP32-S3 is natively little-endian)

    struct os_mbuf *om = ble_hs_mbuf_from_flat(payload, sizeof(payload));
    ble_gatts_notify_custom(s_conn_handle, s_val_feature_vector, om);
}

void ble_gatt_server_update_wear_detection(float current_temp_c)
{
    if (s_state == STATE_MONITORING && current_temp_c < WEAR_ENTER_NOT_WORN_C) {
        set_state(STATE_NOT_WORN, 0);
    } else if (s_state == STATE_NOT_WORN && current_temp_c > WEAR_RETURN_MONITOR_C) {
        set_state(STATE_MONITORING, 0);
    }
}

// =====================================================================
// GATT characteristic access callback — handles all READs and the
// Control Command WRITE.
// =====================================================================
static int gatt_svr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    const ble_uuid_t *uuid = ctxt->chr->uuid;

    if (ble_uuid_cmp(uuid, &CHR_DEVICE_STATE_UUID.u) == 0) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            uint8_t payload[2];
            int len;
            if (s_state == STATE_CALIBRATING || s_state == STATE_ERROR) {
                payload[0] = (uint8_t)s_state; payload[1] = s_aux_byte; len = 2;
            } else {
                payload[0] = (uint8_t)s_state; len = 1;
            }
            os_mbuf_append(ctxt->om, payload, len);
            return 0;
        }
    } else if (ble_uuid_cmp(uuid, &CHR_CONTROL_COMMAND_UUID.u) == 0) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            uint8_t cmd = 0;
            uint16_t out_len = 0;
            ble_hs_mbuf_to_flat(ctxt->om, &cmd, sizeof(cmd), &out_len);
            if (handle_control_command(cmd) != 0) {
                return BLE_ATT_ERR_UNLIKELY;  // firmware state guard rejected it
            }
            return 0;
        }
    } else if (ble_uuid_cmp(uuid, &CHR_CALIBRATION_STATS_UUID.u) == 0) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            uint8_t payload[104];
            memcpy(&payload[0], s_cal_mean, sizeof(s_cal_mean));
            memcpy(&payload[52], s_cal_std, sizeof(s_cal_std));
            os_mbuf_append(ctxt->om, payload, sizeof(payload));
            return 0;
        }
    } else if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(BATTERY_LEVEL_CHR_UUID)) == 0) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            // NOT YET IMPLEMENTED: no battery-sense circuit or driver
            // exists in this project (circuit_design_handoff.md's power
            // supply section doesn't define a voltage-sense GPIO).
            // Returning a fixed placeholder rather than a fabricated
            // reading. Flagged clearly rather than silently guessed.
            uint8_t placeholder = 100;
            os_mbuf_append(ctxt->om, &placeholder, 1);
            return 0;
        }
    }

    return BLE_ATT_ERR_UNLIKELY;
}

// =====================================================================
// GATT service table
// =====================================================================
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &SVC_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &CHR_DEVICE_STATE_UUID.u,
                .access_cb = gatt_svr_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_val_device_state,
            },
            {
                .uuid = &CHR_CONTROL_COMMAND_UUID.u,
                .access_cb = gatt_svr_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE,
                .val_handle = &s_val_control_command,
            },
            {
                .uuid = &CHR_INFERENCE_OUTPUT_UUID.u,
                .access_cb = gatt_svr_access_cb,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_val_inference_output,
            },
            {
                .uuid = &CHR_FEATURE_VECTOR_UUID.u,
                .access_cb = gatt_svr_access_cb,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_val_feature_vector,
            },
            {
                .uuid = &CHR_CALIBRATION_STATS_UUID.u,
                .access_cb = gatt_svr_access_cb,
                .flags = BLE_GATT_CHR_F_READ,
                .val_handle = &s_val_calibration_stats,
            },
            { 0 }  // terminator
        },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BATTERY_SVC_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(BATTERY_LEVEL_CHR_UUID),
                .access_cb = gatt_svr_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_val_battery_level,
            },
            { 0 }
        },
    },
    { 0 }  // terminator
};

// =====================================================================
// GAP event handling — connection, disconnect, subscription changes.
// =====================================================================
static void start_advertising(void);

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                s_conn_handle = event->connect.conn_handle;
                ESP_LOGI(TAG, "BLE client connected");
            } else {
                start_advertising();
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "BLE client disconnected — resubscription will be required on reconnect");
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            s_sub_device_state = s_sub_inference_output = s_sub_feature_vector = s_sub_battery_level = false;
            start_advertising();
            return 0;

        case BLE_GAP_EVENT_SUBSCRIBE:
            if (event->subscribe.attr_handle == s_val_device_state) {
                s_sub_device_state = event->subscribe.cur_notify;
            } else if (event->subscribe.attr_handle == s_val_inference_output) {
                s_sub_inference_output = event->subscribe.cur_notify;
            } else if (event->subscribe.attr_handle == s_val_feature_vector) {
                s_sub_feature_vector = event->subscribe.cur_notify;
            } else if (event->subscribe.attr_handle == s_val_battery_level) {
                s_sub_battery_level = event->subscribe.cur_notify;
            }
            return 0;

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "MTU negotiated: %d bytes", event->mtu.value);
            return 0;

        default:
            return 0;
    }
}

static uint8_t s_own_addr_type = BLE_OWN_ADDR_PUBLIC;  // corrected by ble_sync_cb() at boot

static void start_advertising(void)
{
    struct ble_gap_adv_params adv_params = {0};
    struct ble_hs_adv_fields fields = {0};
    struct ble_hs_adv_fields scan_rsp_fields = {0};

    // BUG FIX (found via real-device testing — app couldn't find the
    // device even after OS-level pairing): this function only ever
    // advertised the device's NAME, never its service UUID. The phone
    // app scans specifically for the service UUID locked in
    // ble_gatt_contract_handoff.md, not just a name match — without
    // the UUID actually being broadcast, no scan filtering on it could
    // ever have found this device, regardless of pairing state.
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    static const ble_uuid128_t adv_uuid = SVC_UUID;
    fields.uuids128 = &adv_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: rc=%d", rc);
        return;
    }

    // A 128-bit UUID plus flags already fills most of a legacy 31-byte
    // advertising packet, so the device name goes into the separate
    // "scan response" packet instead — the standard way to fit more
    // than 31 bytes of advertising data in total.
    const char *name = ble_svc_gap_device_name();
    scan_rsp_fields.name = (uint8_t *)name;
    scan_rsp_fields.name_len = strlen(name);
    scan_rsp_fields.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&scan_rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_rsp_set_fields failed: rc=%d", rc);
        return;
    }

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, gap_event_cb, NULL);
}

static void ble_sync_cb(void)
{
    // BUG FIX (found via real-hardware testing): ble_hs_id_infer_auto()
    // writes its answer INTO the pointer given as its second argument —
    // this chip needs to be told whether to advertise using a "public"
    // or a "random" Bluetooth address, and this function's whole job is
    // figuring that out. Passing NULL here meant it had nowhere to
    // write that answer, so the moment it tried, it crashed writing to
    // memory address 0 — confirmed directly by EXCVADDR: 0x00000000 in
    // the real crash log. The result is now captured in a real
    // variable and actually used below when advertising starts.
    int rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: rc=%d — cannot start advertising", rc);
        return;
    }
    start_advertising();
}

static void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t ble_gatt_server_init(void)
{
    esp_err_t err = nvs_open("nervguard", NVS_READWRITE, &s_nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(err));
        return err;
    }
    load_calibration_from_nvs();

    const esp_timer_create_args_t reset_timer_args = {
        .callback = &reset_timeout_cb,
        .name = "ble_reset_timeout",
    };
    esp_timer_create(&reset_timer_args, &s_reset_timer);

    err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(err));
        return err;
    }

    ble_hs_cfg.sync_cb = ble_sync_cb;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set("NervGuard");

    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);

    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "BLE GATT server ready, advertising as \"NervGuard\", boot state 0x%02X", s_state);
    return ESP_OK;
}