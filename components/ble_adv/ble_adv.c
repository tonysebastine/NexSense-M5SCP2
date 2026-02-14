/*
 * NexSense M5-DoorSensor — BLE Encrypted Advertisement
 *
 * Uses NimBLE in broadcaster-only mode with non-connectable
 * undirected advertising. Payload is AES-128 CCM encrypted
 * and packed into manufacturer-specific data.
 *
 * Payload layout (27 bytes in manufacturer data):
 *   [DeviceID  4B]   - Unique device identifier (last 4 bytes of MAC)
 *   [EventID   4B]   - Monotonic event counter
 *   [EventType 1B]   - OPEN/CLOSED/HEARTBEAT/BAT_LOW
 *   [Battery   2B]   - Battery voltage in mV (little-endian)
 *   [Timestamp 4B]   - Seconds since boot (little-endian)
 *   [Nonce     4B]   - Random nonce for CCM
 *   [AuthTag   8B]   - Truncated AES-CCM authentication tag
 *
 * The first 15 bytes (DeviceID through Timestamp) are the plaintext
 * that gets authenticated (AAD). The AuthTag provides integrity.
 * We use "authenticate-only" mode (no encryption of payload content)
 * since the door state is not confidential — we only need to prevent
 * spoofing/replay.
 */

#include "ble_adv.h"

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include <string.h>

/* NimBLE includes */
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

/* AES-CCM */
#include "mbedtls/ccm.h"

/* ---------- Constants ---------- */

static const char *TAG = "ble_adv";

/* NexSense manufacturer ID (using 0xFFFF = testing/development) */
#define MANUFACTURER_ID 0xFFFF

/* Payload sizes */
#define PAYLOAD_DATA_LEN                                                       \
  11 /* EventID(4) + Type(1) + Bat(2) + TS(4) - DeviceID removed */
#define NONCE_LEN 4
#define AUTH_TAG_LEN 8
#define TOTAL_PAYLOAD_LEN                                                      \
  (PAYLOAD_DATA_LEN + NONCE_LEN + AUTH_TAG_LEN) /* 23 bytes */

/* CCM requires a 7-13 byte nonce; we pad our 4-byte nonce with zeros */
#define CCM_NONCE_LEN 13
#define CCM_TAG_LEN 8

/* ---------- State ---------- */

static uint32_t s_device_id = 0;
static uint8_t s_aes_key[BLE_AES_KEY_LEN] = {
    /* Default development key — MUST be replaced via provisioning */
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
static bool s_ble_ready = false;

/* ---------- NimBLE Callbacks ---------- */

static void ble_on_sync(void) {
  ESP_LOGI(TAG, "BLE host synced");
  s_ble_ready = true;
}

static void ble_on_reset(int reason) {
  ESP_LOGW(TAG, "BLE host reset, reason=%d", reason);
  s_ble_ready = false;
}

static void nimble_host_task(void *param) {
  ESP_LOGI(TAG, "NimBLE host task started");
  nimble_port_run(); /* Returns only on nimble_port_stop() */
  nimble_port_freertos_deinit();
}

/* ---------- Helpers ---------- */

static void derive_device_id(void) {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT);
  /* Use last 4 bytes of BT MAC as device ID */
  s_device_id = ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) |
                ((uint32_t)mac[4] << 8) | ((uint32_t)mac[5]);
  ESP_LOGI(TAG, "Device ID: 0x%08lX", (unsigned long)s_device_id);
}

static void pack_le16(uint8_t *buf, uint16_t val) {
  buf[0] = (uint8_t)(val & 0xFF);
  buf[1] = (uint8_t)((val >> 8) & 0xFF);
}

static void pack_le32(uint8_t *buf, uint32_t val) {
  buf[0] = (uint8_t)(val & 0xFF);
  buf[1] = (uint8_t)((val >> 8) & 0xFF);
  buf[2] = (uint8_t)((val >> 16) & 0xFF);
  buf[3] = (uint8_t)((val >> 24) & 0xFF);
}

/**
 * Build the payload and compute AES-CCM authentication tag.
 *
 *  data[0..10]  = plaintext (EventID, Type, Battery, Timestamp)
 *  data[11..14] = nonce (4 bytes, random)
 *  data[15..22] = auth tag (8 bytes)
 */
static esp_err_t build_encrypted_payload(const event_t *evt,
                                         uint8_t out[TOTAL_PAYLOAD_LEN]) {
  /* Pack plaintext (DeviceID removed to save space) */
  pack_le32(&out[0], evt->event_id);
  out[4] = evt->event_type;
  pack_le16(&out[5], evt->battery_mv);
  pack_le32(&out[7], evt->timestamp);

  /* Generate random nonce */
  uint32_t nonce_val = esp_random();
  pack_le32(&out[11], nonce_val);

  /* Build full CCM nonce (13 bytes): 4-byte random + 4-byte device_id + 5 zeros
   */
  uint8_t ccm_nonce[CCM_NONCE_LEN];
  memset(ccm_nonce, 0, CCM_NONCE_LEN);
  pack_le32(&ccm_nonce[0], nonce_val);
  pack_le32(&ccm_nonce[4], s_device_id);
  /* Remaining 5 bytes stay zero */

  /* Compute AES-CCM auth tag over the plaintext (AAD-only, no ciphertext) */
  mbedtls_ccm_context ctx;
  mbedtls_ccm_init(&ctx);

  int ret = mbedtls_ccm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, s_aes_key,
                               BLE_AES_KEY_LEN * 8);
  if (ret != 0) {
    ESP_LOGE(TAG, "CCM setkey failed: -0x%04X", -ret);
    mbedtls_ccm_free(&ctx);
    return ESP_FAIL;
  }

  uint8_t tag[CCM_TAG_LEN];

  /*
   * mbedtls_ccm_encrypt_and_tag:
   *   - No plaintext to encrypt (length=0, NULL input/output)
   *   - 15 bytes of AAD (Event data 11B + Nonce 4B)
   *   - 8-byte tag output
   */
  ret = mbedtls_ccm_encrypt_and_tag(&ctx, 0, /* plaintext length */
                                    ccm_nonce, CCM_NONCE_LEN, out,
                                    PAYLOAD_DATA_LEN + NONCE_LEN, /* AAD */
                                    NULL, NULL, /* no plaintext */
                                    tag, CCM_TAG_LEN);

  mbedtls_ccm_free(&ctx);

  if (ret != 0) {
    ESP_LOGE(TAG, "CCM encrypt failed: -0x%04X", -ret);
    return ESP_FAIL;
  }

  /* Append tag to payload at offset 15 (11 data + 4 nonce) */
  memcpy(&out[PAYLOAD_DATA_LEN + NONCE_LEN], tag, CCM_TAG_LEN);

  return ESP_OK;
}

/* ---------- API ---------- */

esp_err_t ble_adv_init(void) {
  esp_err_t ret;

  derive_device_id();

  /* Initialize NimBLE */
  ret = nimble_port_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "NimBLE port init failed: %s", esp_err_to_name(ret));
    return ret;
  }

  /* Configure host callbacks */
  ble_hs_cfg.sync_cb = ble_on_sync;
  ble_hs_cfg.reset_cb = ble_on_reset;

  /* Start NimBLE host task */
  nimble_port_freertos_init(nimble_host_task);

  /* Wait for sync (max 2 seconds) */
  for (int i = 0; i < 20 && !s_ble_ready; i++) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  if (!s_ble_ready) {
    ESP_LOGE(TAG, "BLE host failed to sync");
    return ESP_ERR_TIMEOUT;
  }

  ESP_LOGI(TAG, "BLE advertisement module initialized");
  return ESP_OK;
}

esp_err_t ble_adv_send_event(const event_t *evt) {
  if (!s_ble_ready) {
    ESP_LOGE(TAG, "BLE not ready");
    return ESP_ERR_INVALID_STATE;
  }

  /* Build encrypted payload */
  uint8_t payload[TOTAL_PAYLOAD_LEN];
  esp_err_t ret = build_encrypted_payload(evt, payload);
  if (ret != ESP_OK) {
    return ret;
  }

  /* Build manufacturer data: [Manufacturer ID (2B LE)] + [payload (27B)] */
  uint8_t mfg_data[2 + TOTAL_PAYLOAD_LEN];
  pack_le16(mfg_data, MANUFACTURER_ID);
  memcpy(&mfg_data[2], payload, TOTAL_PAYLOAD_LEN);

  /* Configure advertisement fields */
  struct ble_hs_adv_fields fields;
  memset(&fields, 0, sizeof(fields));

  fields.flags = BLE_HS_ADV_F_BREDR_UNSUP; /* BLE-only */
  fields.mfg_data = mfg_data;
  fields.mfg_data_len = sizeof(mfg_data);

  int rc = ble_gap_adv_set_fields(&fields);
  if (rc != 0) {
    ESP_LOGE(TAG, "Failed to set adv fields: rc=%d", rc);
    return ESP_FAIL;
  }

  /* Configure non-connectable undirected advertising */
  struct ble_gap_adv_params adv_params;
  memset(&adv_params, 0, sizeof(adv_params));
  adv_params.conn_mode = BLE_GAP_CONN_MODE_NON; /* Non-connectable */
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; /* General discoverable */
  adv_params.itvl_min = BLE_ADV_INTERVAL_MIN;
  adv_params.itvl_max = BLE_ADV_INTERVAL_MAX;

  /* Start advertising */
  rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params,
                         NULL, NULL);
  if (rc != 0) {
    ESP_LOGE(TAG, "Failed to start adv: rc=%d", rc);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Broadcasting event #%lu (%s) for %dms",
           (unsigned long)evt->event_id, event_type_to_str(evt->event_type),
           BLE_ADV_BURST_MS);

  /* Burst: advertise for the configured duration */
  vTaskDelay(pdMS_TO_TICKS(BLE_ADV_BURST_MS));

  /* Stop advertising */
  ble_gap_adv_stop();

  return ESP_OK;
}

esp_err_t ble_adv_stop(void) {
  if (ble_gap_adv_active()) {
    ble_gap_adv_stop();
  }
  return ESP_OK;
}

uint32_t ble_adv_get_device_id(void) { return s_device_id; }

void ble_adv_set_key(const uint8_t key[BLE_AES_KEY_LEN]) {
  memcpy(s_aes_key, key, BLE_AES_KEY_LEN);
  ESP_LOGI(TAG, "AES key updated");
}
