/*
 * NexSense M5-DoorSensor — State Engine
 *
 * Evaluates door state from accelerometer readings using
 * Euclidean distance to calibrated references with
 * N-sample confirmation and movement lockout.
 */

#pragma once

#include "esp_err.h"
#include "sensor.h"
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Types ---------- */

typedef enum {
  DOOR_UNKNOWN = 0,
  DOOR_OPEN = 1,
  DOOR_CLOSED = 2,
} door_state_t;

typedef struct {
  door_state_t current_state; /* Last committed state                    */
  door_state_t pending_state; /* State being confirmed                   */
  uint8_t confirm_count;      /* Consecutive samples matching pending    */
  int64_t lockout_until_us;   /* Timestamp: ignore changes until this    */
  float threshold;            /* Euclidean distance threshold            */
  uint8_t required_confirms;  /* N-sample confirmation count             */
  uint32_t lockout_ms;        /* Movement lockout duration               */
} state_engine_ctx_t;

/* ---------- API ---------- */

/**
 * @brief Initialize the state engine.
 * @return ESP_OK on success.
 */
esp_err_t state_engine_init(void);

/**
 * @brief Evaluate a new accelerometer reading against calibration.
 *
 * @param[in] accel  Normalized 3-axis accelerometer reading.
 * @param[out] state_changed  Set to true if committed state changed.
 * @return The current committed door state.
 */
door_state_t state_engine_evaluate(const float accel[3], bool *state_changed);

/**
 * @brief Get the current committed door state.
 */
door_state_t state_engine_get_state(void);

/**
 * @brief Set the detection threshold (default 0.15).
 */
void state_engine_set_threshold(float threshold);

/**
 * @brief Get the string name of a door state.
 */
const char *door_state_to_str(door_state_t state);

#ifdef __cplusplus
}
#endif
