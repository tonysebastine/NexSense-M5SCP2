/*
 * NexSense M5-DoorSensor — Main Application
 * Phase 1: Core Firmware
 *
 * Target: M5StickC Plus 2 (ESP32-PICO-V3-02)
 *
 * Boot flow:
 *   1. Initialize NVS, all components
 *   2. Show boot screen (2 seconds)
 *   3. Load calibration from NVS
 *   4. If not calibrated → prompt user
 *   5. Main loop:
 *      - Read IMU → normalize → evaluate state
 *      - If state changed → create event → BLE broadcast
 *      - Periodically send heartbeat
 *      - Check battery
 *      - Handle button presses (calibration, display wake)
 *      - Enter light sleep
 */

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

/* Components */
#include "ble_adv.h"
#include "event_mgr.h"
#include "power_mgr.h"
#include "sensor.h"
#include "state_engine.h"
#include "ui.h"

/* ---------- Constants ---------- */

static const char *TAG = "main";
#define FW_VERSION "1.0.0-alpha"

/* Buttons */
#define BTN_A_PIN GPIO_NUM_37 /* Front button  */
#define BTN_B_PIN GPIO_NUM_39 /* Side button   */

/* Timing */
#define BOOT_SCREEN_MS 2000
#define HEARTBEAT_INTERVAL_S 300 /* Heartbeat every 5 minutes */
#define MAIN_LOOP_DELAY_MS 100   /* Sampling interval in active mode */
#define BTN_LONG_PRESS_MS 2000   /* Long press threshold */
#define BTN_DEBOUNCE_MS 50

/* Calibration states */
typedef enum {
  CAL_IDLE = 0,
  CAL_WAIT_OPEN,      /* Waiting for user to position door OPEN */
  CAL_CAPTURE_OPEN,   /* Capturing OPEN reference */
  CAL_WAIT_CLOSED,    /* Waiting for user to position door CLOSED */
  CAL_CAPTURE_CLOSED, /* Capturing CLOSED reference */
  CAL_DONE,
} cal_mode_t;

/* ---------- State ---------- */

static calibration_t s_cal;
static RTC_DATA_ATTR uint32_t s_last_heartbeat_ts = 0;
static bool s_display_active = true;
static cal_mode_t s_cal_mode = CAL_IDLE;
static bool s_display_dirty = true; /* Force first draw */

/* ---------- Button Handling ---------- */

typedef struct {
  gpio_num_t pin;
  int64_t press_start_us;
  bool pressed;
  bool long_fired;
} button_t;

static button_t s_btn_a = {.pin = BTN_A_PIN};
static button_t s_btn_b = {.pin = BTN_B_PIN};

static void button_update(button_t *btn) {
  bool level = (gpio_get_level(btn->pin) == 0); /* Active low */

  if (level && !btn->pressed) {
    /* Button just pressed */
    btn->pressed = true;
    btn->press_start_us = esp_timer_get_time();
    btn->long_fired = false;
  } else if (!level && btn->pressed) {
    /* Button released */
    btn->pressed = false;
  }

  /* Check for long press */
  if (btn->pressed && !btn->long_fired) {
    int64_t held = esp_timer_get_time() - btn->press_start_us;
    if (held > (int64_t)BTN_LONG_PRESS_MS * 1000LL) {
      btn->long_fired = true;
    }
  }
}

static bool button_long_press(button_t *btn) { return btn->long_fired; }

static bool button_short_press(button_t *btn) {
  /* Detected on release if it was a short press */
  if (!btn->pressed && btn->press_start_us > 0 && !btn->long_fired) {
    int64_t held = esp_timer_get_time() - btn->press_start_us;
    if (held > (int64_t)BTN_DEBOUNCE_MS * 1000LL) {
      btn->press_start_us = 0; /* Consume the event */
      return true;
    }
  }
  return false;
}

/* ---------- Calibration Flow ---------- */

static void handle_calibration(void) {
  switch (s_cal_mode) {
  case CAL_WAIT_OPEN:
    ui_show_calibration("OPEN POS");
    /* Wait for Button A short press to capture */
    if (button_short_press(&s_btn_a)) {
      s_cal_mode = CAL_CAPTURE_OPEN;
    }
    break;

  case CAL_CAPTURE_OPEN:
    ui_show_message("SAMPLING", COLOR_YELLOW);
    sensor_calibrate_open(&s_cal);
    ui_show_message("OPEN OK!", COLOR_GREEN);
    vTaskDelay(pdMS_TO_TICKS(1000));
    s_cal_mode = CAL_WAIT_CLOSED;
    break;

  case CAL_WAIT_CLOSED:
    ui_show_calibration("CLOSE POS");
    if (button_short_press(&s_btn_a)) {
      s_cal_mode = CAL_CAPTURE_CLOSED;
    }
    break;

  case CAL_CAPTURE_CLOSED:
    ui_show_message("SAMPLING", COLOR_YELLOW);
    sensor_calibrate_closed(&s_cal);

    /* Save to NVS */
    if (sensor_save_calibration(&s_cal) == ESP_OK) {
      ui_show_message("SAVED!", COLOR_GREEN);
    } else {
      ui_show_message("SAVE ERR", COLOR_RED);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    s_cal_mode = CAL_DONE;
    break;

  case CAL_DONE:
    s_cal_mode = CAL_IDLE;
    sensor_enable_gyro(false); /* Disable gyro to save power */
    ESP_LOGI(TAG, "Calibration complete");
    break;

  default:
    break;
  }
}

/* ---------- Heartbeat ---------- */

static void check_heartbeat(uint16_t battery_mv) {
  uint32_t now_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);

  if ((now_s - s_last_heartbeat_ts) >= HEARTBEAT_INTERVAL_S) {
    s_last_heartbeat_ts = now_s;

    event_t evt = event_mgr_create(EVENT_HEARTBEAT, battery_mv);
    ble_adv_send_event(&evt);

    ESP_LOGI(TAG, "Heartbeat sent (event #%lu)", (unsigned long)evt.event_id);
  }
}

/* ---------- Battery Check ---------- */

static void check_battery(uint16_t battery_mv) {
  static bool s_bat_low_sent = false;
  static int64_t s_last_blink_us = 0;

  if (power_mgr_is_battery_low()) {
    /* Low battery alert */
    if (!s_bat_low_sent) {
      event_t evt = event_mgr_create(EVENT_BAT_LOW, battery_mv);
      ble_adv_send_event(&evt);
      s_bat_low_sent = true;
      ESP_LOGW(TAG, "Low battery alert sent (%umV)", battery_mv);
    }

    /* Double blink every 10 seconds to indicate low battery without staying
     * "on" */
    int64_t now_us = esp_timer_get_time();
    if ((now_us - s_last_blink_us) > 10000000LL) {
      power_mgr_set_led(true);
      vTaskDelay(pdMS_TO_TICKS(20));
      power_mgr_set_led(false);
      vTaskDelay(pdMS_TO_TICKS(100));
      power_mgr_set_led(true);
      vTaskDelay(pdMS_TO_TICKS(20));
      power_mgr_set_led(false);
      s_last_blink_us = now_us;
    }
  } else {
    s_bat_low_sent = false;
  }
}

/* ---------- Main ---------- */

void app_main(void) {
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "  NexSense M5-DoorSensor v%s", FW_VERSION);
  ESP_LOGI(TAG, "========================================");

  /* --- Initialize NVS --- */
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(TAG, "NVS partition truncated, erasing...");
    nvs_flash_erase();
    nvs_flash_init();
  }

  /* --- Initialize Components --- */
  ESP_LOGI(TAG, "Initializing sensor...");
  ESP_ERROR_CHECK(sensor_init());

  ESP_LOGI(TAG, "Initializing power manager...");
  ESP_ERROR_CHECK(power_mgr_init());

  ESP_LOGI(TAG, "Initializing UI...");
  ret = ui_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "UI init FAILED: %s", esp_err_to_name(ret));
  }

  ESP_LOGI(TAG, "Initializing state engine...");
  ret = state_engine_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "State engine init FAILED: %s", esp_err_to_name(ret));
  }

  ESP_LOGI(TAG, "Initializing event manager...");
  ret = event_mgr_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Event manager init FAILED: %s", esp_err_to_name(ret));
  }

  ESP_LOGI(TAG, "Initializing BLE...");
  ret = ble_adv_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "BLE init FAILED: %s", esp_err_to_name(ret));
  }

  /* --- Boot Screen --- */
  uint32_t device_id = ble_adv_get_device_id();

  /* Enable Power Management for DFS (Dynamic Frequency Scaling) */
  esp_pm_config_esp32_t pm_config = {
      .max_freq_mhz = 240,         /* Max speed for NimBLE/AES */
      .min_freq_mhz = 40,          /* Min speed for polling loop */
      .light_sleep_enable = false, /* We handle light sleep manually */
  };
  esp_pm_configure(&pm_config);

  ui_show_boot(device_id, FW_VERSION);
  ESP_LOGI(TAG, "Device ID: 0x%08lX", (unsigned long)device_id);
  vTaskDelay(pdMS_TO_TICKS(BOOT_SCREEN_MS));

  /* --- Load Calibration --- */
  bool calibrated = false;
  ret = sensor_load_calibration(&s_cal);
  if (ret == ESP_OK && s_cal.valid) {
    calibrated = true;
    ESP_LOGI(TAG, "Calibration loaded successfully");
    ESP_LOGI(TAG, "  OPEN ref:   [%.3f, %.3f, %.3f]", s_cal.open_ref[0],
             s_cal.open_ref[1], s_cal.open_ref[2]);
    ESP_LOGI(TAG, "  CLOSED ref: [%.3f, %.3f, %.3f]", s_cal.closed_ref[0],
             s_cal.closed_ref[1], s_cal.closed_ref[2]);
  } else {
    ESP_LOGW(TAG, "No calibration found — device needs calibration");
  }

  /* --- Send boot heartbeat --- */
  uint16_t battery_mv = power_mgr_get_battery_mv();
  {
    event_t boot_evt = event_mgr_create(EVENT_HEARTBEAT, battery_mv);
    ble_adv_send_event(&boot_evt);
    s_last_heartbeat_ts = (uint32_t)(esp_timer_get_time() / 1000000ULL);
  }

  /* --- Boot Feedback --- */
  power_mgr_set_led(true);
  power_mgr_beep(50);
  vTaskDelay(pdMS_TO_TICKS(100));
  power_mgr_set_led(false);

  ESP_LOGI(TAG, "Entering main loop (battery=%umV)", battery_mv);

  /* ======================== MAIN LOOP ======================== */
  while (1) {
    /* Update button states */
    button_update(&s_btn_a);
    button_update(&s_btn_b);

    /* --- Button B short press: wake display --- */
    if (button_short_press(&s_btn_b)) {
      if (!ui_is_display_on()) {
        ui_set_display(true);
        s_display_active = true;
        s_display_dirty = true;
      }
    }

    /* --- Button A long press: enter calibration mode --- */
    if (button_long_press(&s_btn_a) && s_cal_mode == CAL_IDLE) {
      ESP_LOGI(TAG, "Entering calibration mode");
      s_cal_mode = CAL_WAIT_OPEN;
      ui_set_display(true);
      s_display_active = true;
      sensor_enable_gyro(true); /* Enable gyro for orientation assistance */
    }

    /* --- Calibration flow --- */
    if (s_cal_mode != CAL_IDLE) {
      handle_calibration();
      if (s_cal_mode == CAL_IDLE) {
        /* Calibration just completed */
        calibrated = s_cal.valid;
      }
      vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_DELAY_MS));
      continue; /* Skip normal processing during calibration */
    }

    /* --- Read IMU --- */
    float accel[3];
    ret = sensor_read_accel(accel);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "IMU read failed");
      vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_DELAY_MS));
      continue;
    }
    sensor_normalize(accel);

    /* --- Evaluate State --- */
    bool state_changed = false;
    door_state_t state = state_engine_evaluate(accel, &state_changed);

    /* --- Handle State Change --- */
    if (state_changed) {
      battery_mv = power_mgr_get_battery_mv();

      uint8_t evt_type = (state == DOOR_OPEN) ? EVENT_OPEN : EVENT_CLOSED;
      event_t evt = event_mgr_create(evt_type, battery_mv);

      ESP_LOGI(TAG, "*** DOOR %s *** (event #%lu)", door_state_to_str(state),
               (unsigned long)evt.event_id);

      /* Broadcast via BLE */
      ble_adv_send_event(&evt);

      /* Visual/Audio Feedback removed for battery efficiency */
      /* power_mgr_set_led(true); ... */

      /* Wake display to show new state */
      ui_set_display(true);
      s_display_active = true;
      s_display_dirty = true;
    }

    /* --- Update Display --- */
    if (s_display_active) {
      /* Throttle battery reads: only every 5 seconds */
      static int64_t s_last_bat_read_us = 0;
      static uint16_t s_cached_bat_mv = 0;
      int64_t now_us = esp_timer_get_time();
      if (s_cached_bat_mv == 0 || (now_us - s_last_bat_read_us) > 5000000LL) {
        s_cached_bat_mv = power_mgr_get_battery_mv();
        s_last_bat_read_us = now_us;
      }
      battery_mv = s_cached_bat_mv;
      uint32_t evt_cnt = event_mgr_get_counter();

      /* Only redraw if something changed */
      static door_state_t s_prev_state = DOOR_UNKNOWN;
      static uint16_t s_prev_bat_pct = 0xFFFF;
      static uint32_t s_prev_evt_cnt = 0xFFFFFFFF;

      /* Quantize to 5% steps to avoid noise-driven redraws */
      uint16_t bat_pct = 0;
      if (battery_mv >= 4200)
        bat_pct = 100;
      else if (battery_mv <= 3300)
        bat_pct = 0;
      else
        bat_pct = ((battery_mv - 3300) * 100 / 900) / 5 * 5;

      if (s_display_dirty || state != s_prev_state ||
          bat_pct != s_prev_bat_pct || evt_cnt != s_prev_evt_cnt) {
        ui_show_state(state, battery_mv, evt_cnt, calibrated);
        s_prev_state = state;
        s_prev_bat_pct = bat_pct;
        s_prev_evt_cnt = evt_cnt;
        s_display_dirty = false;
      }

      /* Check auto-off */
      if (!ui_is_display_on()) {
        s_display_active = false;
      }
    }

    /* --- Heartbeat --- */
    check_heartbeat(battery_mv);

    /* --- Battery Check --- */
    check_battery(battery_mv);

    /* --- Sleep / Delay --- */
    if (!s_display_active && s_cal_mode == CAL_IDLE) {
      /* Display is off: Switch sensor to LP mode and sleep adaptively */
      sensor_set_low_power(true);

      uint32_t sleep_ms = 200; /* Faster wake when door is OPEN */
      if (state == DOOR_CLOSED) {
        sleep_ms = 1000; /* Power saving when DOOR is stable CLOSED */
      }

      power_mgr_enter_sleep(sleep_ms);

      /* On wakeup, restore sensor to normal mode for processing */
      sensor_set_low_power(false);
    } else {
      /* Active mode: use FreeRTOS delay */
      vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_DELAY_MS));
    }
  }
}
