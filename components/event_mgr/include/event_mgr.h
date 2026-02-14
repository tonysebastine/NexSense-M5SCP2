/*
 * NexSense M5-DoorSensor — Event Manager
 *
 * Monotonically increasing event IDs with RTC + NVS persistence.
 * Store-before-send guarantee for security-grade reliability.
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Event Types ---------- */

#define EVENT_OPEN 0x01
#define EVENT_CLOSED 0x02
#define EVENT_HEARTBEAT 0x03
#define EVENT_BAT_LOW 0x04

/* ---------- Types ---------- */

typedef struct {
  uint32_t event_id;   /* Monotonically increasing ID                */
  uint8_t event_type;  /* EVENT_OPEN, EVENT_CLOSED, etc.             */
  uint16_t battery_mv; /* Battery voltage in millivolts              */
  uint32_t timestamp;  /* Seconds since boot (from esp_timer)        */
} __attribute__((packed)) event_t;

/* ---------- API ---------- */

/**
 * @brief Initialize the event manager.
 * Restores event_id from RTC / NVS.
 * @return ESP_OK on success.
 */
esp_err_t event_mgr_init(void);

/**
 * @brief Create a new event with the next monotonic ID.
 * @param[in] type       Event type (EVENT_OPEN, etc.)
 * @param[in] battery_mv Battery voltage in mV.
 * @return Populated event_t.
 */
event_t event_mgr_create(uint8_t type, uint16_t battery_mv);

/**
 * @brief Checkpoint the current event_id to NVS.
 * Called periodically (every 100 events or 1 hour).
 */
esp_err_t event_mgr_checkpoint(void);

/**
 * @brief Get the current event counter value.
 */
uint32_t event_mgr_get_counter(void);

/**
 * @brief Get the string name of an event type.
 */
const char *event_type_to_str(uint8_t type);

#ifdef __cplusplus
}
#endif
