/*
 * NexSense M5-DoorSensor — Event Manager
 *
 * Maintains a monotonically increasing event counter across
 * sleep cycles (RTC memory) with periodic NVS checkpointing.
 */

#include "event_mgr.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <string.h>


/* ---------- Constants ---------- */

static const char *TAG = "event_mgr";

#define NVS_NAMESPACE "doorsensor"
#define NVS_KEY_EVT_ID "evt_id"
#define CHECKPOINT_INTERVAL 100 /* Checkpoint every N events */

/* ---------- RTC State ---------- */

static RTC_DATA_ATTR uint32_t s_event_id = 0;
static RTC_DATA_ATTR uint32_t s_last_checkpoint_id = 0;
static RTC_DATA_ATTR bool s_evt_initialized = false;

/* ---------- Initialization ---------- */

esp_err_t event_mgr_init(void) {
  if (s_evt_initialized) {
    ESP_LOGI(TAG, "Event manager restored from RTC (event_id=%lu)",
             (unsigned long)s_event_id);
    return ESP_OK;
  }

  /* Try to restore from NVS */
  nvs_handle_t nvs;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
  if (ret == ESP_OK) {
    uint32_t stored_id = 0;
    ret = nvs_get_u32(nvs, NVS_KEY_EVT_ID, &stored_id);
    if (ret == ESP_OK) {
      s_event_id = stored_id;
      s_last_checkpoint_id = stored_id;
      ESP_LOGI(TAG, "Event ID restored from NVS: %lu",
               (unsigned long)s_event_id);
    } else {
      ESP_LOGI(TAG, "No stored event ID, starting from 0");
      s_event_id = 0;
    }
    nvs_close(nvs);
  } else {
    ESP_LOGW(TAG, "NVS open failed, starting event ID from 0");
    s_event_id = 0;
  }

  s_evt_initialized = true;
  return ESP_OK;
}

/* ---------- Event Creation ---------- */

event_t event_mgr_create(uint8_t type, uint16_t battery_mv) {
  event_t evt;
  evt.event_id = ++s_event_id;
  evt.event_type = type;
  evt.battery_mv = battery_mv;
  evt.timestamp = (uint32_t)(esp_timer_get_time() / 1000000ULL);

  ESP_LOGI(TAG, "Event #%lu: type=%s, bat=%umV", (unsigned long)evt.event_id,
           event_type_to_str(type), evt.battery_mv);

  /* Auto-checkpoint if interval reached */
  if ((s_event_id - s_last_checkpoint_id) >= CHECKPOINT_INTERVAL) {
    event_mgr_checkpoint();
  }

  return evt;
}

/* ---------- Checkpointing ---------- */

esp_err_t event_mgr_checkpoint(void) {
  nvs_handle_t nvs;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "NVS open failed for checkpoint: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = nvs_set_u32(nvs, NVS_KEY_EVT_ID, s_event_id);
  if (ret == ESP_OK) {
    ret = nvs_commit(nvs);
  }
  nvs_close(nvs);

  if (ret == ESP_OK) {
    s_last_checkpoint_id = s_event_id;
    ESP_LOGI(TAG, "Checkpoint: event_id=%lu saved to NVS",
             (unsigned long)s_event_id);
  }

  return ret;
}

uint32_t event_mgr_get_counter(void) { return s_event_id; }

const char *event_type_to_str(uint8_t type) {
  switch (type) {
  case EVENT_OPEN:
    return "OPEN";
  case EVENT_CLOSED:
    return "CLOSED";
  case EVENT_HEARTBEAT:
    return "HEARTBEAT";
  case EVENT_BAT_LOW:
    return "BAT_LOW";
  default:
    return "UNKNOWN";
  }
}
