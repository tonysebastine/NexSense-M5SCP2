/*
 * NexSense M5-DoorSensor — State Engine
 *
 * Euclidean distance evaluation against calibrated OPEN/CLOSED references.
 * Security-grade: requires N consecutive confirmations and enforces
 * a movement lockout window before committing state transitions.
 */

#include "state_engine.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "sensor.h"

/* ---------- Constants ---------- */

static const char *TAG = "state_engine";

#define DEFAULT_THRESHOLD 0.15f
#define DEFAULT_CONFIRMS 3
#define DEFAULT_LOCKOUT_MS 300

/* ---------- State ---------- */

/* Persisted in RTC for fast access after light sleep */
static RTC_DATA_ATTR state_engine_ctx_t s_ctx;
static RTC_DATA_ATTR bool s_initialized = false;

/* ---------- Helpers ---------- */

static float euclidean_distance(const float a[3], const float b[3]) {
  float dx = a[0] - b[0];
  float dy = a[1] - b[1];
  float dz = a[2] - b[2];
  return sqrtf(dx * dx + dy * dy + dz * dz);
}

/* ---------- API ---------- */

esp_err_t state_engine_init(void) {
  if (!s_initialized) {
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.current_state = DOOR_UNKNOWN;
    s_ctx.pending_state = DOOR_UNKNOWN;
    s_ctx.confirm_count = 0;
    s_ctx.lockout_until_us = 0;
    s_ctx.threshold = DEFAULT_THRESHOLD;
    s_ctx.required_confirms = DEFAULT_CONFIRMS;
    s_ctx.lockout_ms = DEFAULT_LOCKOUT_MS;
    s_initialized = true;
    ESP_LOGI(
        TAG,
        "State engine initialized (threshold=%.2f, confirms=%d, lockout=%lums)",
        s_ctx.threshold, s_ctx.required_confirms,
        (unsigned long)s_ctx.lockout_ms);
  } else {
    ESP_LOGI(TAG, "State engine restored from RTC (state=%s)",
             door_state_to_str(s_ctx.current_state));
  }
  return ESP_OK;
}

door_state_t state_engine_evaluate(const float accel[3], bool *state_changed) {
  *state_changed = false;

  const calibration_t *cal = sensor_get_calibration();
  if (!cal || !cal->valid) {
    return DOOR_UNKNOWN;
  }

  int64_t now_us = esp_timer_get_time();

  /* Movement lockout: ignore readings during lockout window */
  if (now_us < s_ctx.lockout_until_us) {
    return s_ctx.current_state;
  }

  /* Compute distances to both references */
  float d_open = euclidean_distance(accel, cal->open_ref);
  float d_closed = euclidean_distance(accel, cal->closed_ref);

  /* Determine candidate state */
  door_state_t candidate;
  if (d_open < s_ctx.threshold) {
    candidate = DOOR_OPEN;
  } else if (d_closed < s_ctx.threshold) {
    candidate = DOOR_CLOSED;
  } else {
    /* Neither reference matched — movement or unknown orientation */
    candidate = DOOR_UNKNOWN;
    /* Reset confirmation counter since we're in an indeterminate state */
    s_ctx.confirm_count = 0;
    s_ctx.pending_state = DOOR_UNKNOWN;
    return s_ctx.current_state;
  }

  /* N-sample confirmation */
  if (candidate == s_ctx.pending_state) {
    s_ctx.confirm_count++;
  } else {
    /* Different candidate: restart confirmation */
    s_ctx.pending_state = candidate;
    s_ctx.confirm_count = 1;
  }

  /* Check if we have enough confirmations for a state commit */
  if (s_ctx.confirm_count >= s_ctx.required_confirms) {
    if (candidate != s_ctx.current_state) {
      ESP_LOGI(TAG, "State transition: %s → %s (d_open=%.3f, d_closed=%.3f)",
               door_state_to_str(s_ctx.current_state),
               door_state_to_str(candidate), d_open, d_closed);

      s_ctx.current_state = candidate;
      *state_changed = true;

      /* Engage movement lockout to prevent bouncing */
      s_ctx.lockout_until_us = now_us + (s_ctx.lockout_ms * 1000LL);
    }
    /* Reset confirmation counter */
    s_ctx.confirm_count = 0;
    s_ctx.pending_state = DOOR_UNKNOWN;
  }

  return s_ctx.current_state;
}

door_state_t state_engine_get_state(void) { return s_ctx.current_state; }

void state_engine_set_threshold(float threshold) {
  if (threshold > 0.01f && threshold < 2.0f) {
    s_ctx.threshold = threshold;
    ESP_LOGI(TAG, "Threshold set to %.2f", threshold);
  }
}

const char *door_state_to_str(door_state_t state) {
  switch (state) {
  case DOOR_OPEN:
    return "OPEN";
  case DOOR_CLOSED:
    return "CLOSED";
  case DOOR_UNKNOWN:
    return "UNKNOWN";
  default:
    return "INVALID";
  }
}
