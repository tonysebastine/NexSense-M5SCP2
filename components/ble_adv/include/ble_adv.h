/*
 * NexSense M5-DoorSensor — BLE Encrypted Advertisement
 *
 * Non-connectable broadcast-only BLE using AES-128 CCM encrypted
 * manufacturer-specific advertisement data.
 */

#pragma once

#include "esp_err.h"
#include "event_mgr.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Configuration ---------- */

#define BLE_ADV_BURST_MS                                                       \
  1500 /* Increased to ensure reliable catch by scanners */
#define BLE_ADV_INTERVAL_MIN 160 /* 100ms in 0.625ms units               */
#define BLE_ADV_INTERVAL_MAX 480 /* 300ms in 0.625ms units               */
#define BLE_AES_KEY_LEN 16       /* AES-128 key length                   */

/* ---------- API ---------- */

/**
 * @brief Initialize BLE subsystem for broadcast-only operation.
 * @return ESP_OK on success.
 */
esp_err_t ble_adv_init(void);

/**
 * @brief Broadcast an event via BLE advertisement.
 * Encrypts the event payload and advertises for BLE_ADV_BURST_MS.
 * This call blocks for the burst duration.
 *
 * @param[in] evt  Event to broadcast.
 * @return ESP_OK on success.
 */
esp_err_t ble_adv_send_event(const event_t *evt);

/**
 * @brief Stop any active advertisement.
 */
esp_err_t ble_adv_stop(void);

/**
 * @brief Get the 32-bit device ID (derived from MAC).
 */
uint32_t ble_adv_get_device_id(void);

/**
 * @brief Set the AES-128 network encryption key.
 * @param[in] key  16-byte AES key.
 */
void ble_adv_set_key(const uint8_t key[BLE_AES_KEY_LEN]);

#ifdef __cplusplus
}
#endif
