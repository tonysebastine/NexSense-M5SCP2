/*
 * NexSense M5-DoorSensor — Power Manager
 *
 * Direct GPIO and ADC control for M5StickC Plus 2.
 * Hardware:
 *   - Power Hold: GPIO 4 (Set HIGH to stay on)
 *   - LCD Backlight: GPIO 27
 *   - Battery ADC: ADC1 Channel 2 (with 2:1 voltage divider)
 *   - Buttons: GPIO 37 (A), GPIO 39 (B)
 */

#include "power_mgr.h"

#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include <string.h>

/* ---------- Constants ---------- */

static const char *TAG = "power_mgr";

/* Pins */
#define PWR_HOLD_PIN GPIO_NUM_4
#define LCD_BK_PIN GPIO_NUM_27
#define BAT_ADC_PIN GPIO_NUM_38
#define BAT_ADC_CHAN ADC_CHANNEL_2
#define RED_LED_PIN GPIO_NUM_19
#define BUZZER_PIN GPIO_NUM_2

#define BTN_A_PIN GPIO_NUM_37
#define BTN_B_PIN GPIO_NUM_39

/* ADC Config */
#define ADC_UNIT ADC_UNIT_1
#define ADC_ATTEN ADC_ATTEN_DB_12

/* Logic */
#define BAT_DIVIDER_RATIO 2.0f

/* ---------- State ---------- */

static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t s_adc_cali_handle = NULL;

/* ---------- Initialization ---------- */

esp_err_t power_mgr_init(void) {
  esp_err_t ret;

  /* 1. POWER HOLD: Set GPIO 4 HIGH immediately to maintain power */
  gpio_config_t hold_conf = {
      .pin_bit_mask = (1ULL << PWR_HOLD_PIN),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&hold_conf);
  gpio_set_level(PWR_HOLD_PIN, 1);
  gpio_hold_en(PWR_HOLD_PIN); /* Maintain through light sleep */
  ESP_LOGI(TAG, "Power hold enabled (GPIO %d HIGH)", PWR_HOLD_PIN);

  /* 2. LCD BACKLIGHT & LED & BUZZER: Initialize as output */
  gpio_config_t out_conf = {
      .pin_bit_mask =
          (1ULL << LCD_BK_PIN) | (1ULL << RED_LED_PIN) | (1ULL << BUZZER_PIN),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&out_conf);
  gpio_set_level(LCD_BK_PIN, 0);  /* Backlight OFF */
  gpio_set_level(RED_LED_PIN, 1); /* LED OFF (Active Low) */
  gpio_set_level(BUZZER_PIN, 0);  /* Buzzer OFF */

  /* 3. BATTERY ADC: Initialize ADC1 with calibration */
  adc_oneshot_unit_init_cfg_t init_config = {
      .unit_id = ADC_UNIT,
      .clk_src = 0,
  };
  ret = adc_oneshot_new_unit(&init_config, &s_adc_handle);
  if (ret == ESP_OK) {
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN,
    };
    adc_oneshot_config_channel(s_adc_handle, BAT_ADC_CHAN, &config);

    /* Set up line-fitting calibration for accurate voltage readings */
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &s_adc_cali_handle);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "ADC calibration not available, using raw conversion");
      s_adc_cali_handle = NULL;
    }
    ESP_LOGI(TAG, "Battery ADC initialized (GPIO %d)", BAT_ADC_PIN);
  } else {
    ESP_LOGE(TAG, "Failed to initialize ADC: %s", esp_err_to_name(ret));
  }

  /* 4. BUTTONS: Configure for wakeup */
  gpio_config_t btn_conf = {
      .pin_bit_mask = (1ULL << BTN_A_PIN) | (1ULL << BTN_B_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE, /* GPIO 37/39 are input-only, ext
                                            pull-up on board */
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&btn_conf);

  ESP_LOGI(TAG, "Power manager initialized (M5StickC Plus 2 Hardware) "
                "[LED:19, BUZ:2]");
  return ESP_OK;
}

/* ---------- Peripherals ---------- */

void power_mgr_set_led(bool on) {
  /* Red LED on M5StickC Plus 2 is Active Low */
  gpio_set_level(RED_LED_PIN, on ? 0 : 1);
}

void power_mgr_beep(uint32_t duration_ms) {
  /* Simple square wave bit-banging for now (4kHz approx) */
  /* For better audio, use LEDC driver phase 2 */
  uint32_t cycles = duration_ms * 4; /* 4 cycles per ms = 4kHz? No, 4000Hz =
                                        4000 cycles/s = 4 cycles/ms */
  for (uint32_t i = 0; i < cycles; i++) {
    gpio_set_level(BUZZER_PIN, 1);
    esp_rom_delay_us(125); /* 125us high */
    gpio_set_level(BUZZER_PIN, 0);
    esp_rom_delay_us(125); /* 125us low -> 250us period = 4kHz */
  }
}

/* ---------- Battery ---------- */

uint16_t power_mgr_get_battery_mv(void) {
  if (s_adc_handle == NULL)
    return 0;

  /* Average 4 samples for noise reduction */
  int raw = 0;
  for (int i = 0; i < 4; i++) {
    int val = 0;
    adc_oneshot_read(s_adc_handle, BAT_ADC_CHAN, &val);
    raw += val;
  }
  raw /= 4;

  /* Convert to voltage using calibration if available */
  int voltage = 0;
  if (s_adc_cali_handle) {
    adc_cali_raw_to_voltage(s_adc_cali_handle, raw, &voltage);
  } else {
    voltage = raw * 3300 / 4095;
  }

  /* Apply voltage divider ratio (2:1) */
  return (uint16_t)(voltage * BAT_DIVIDER_RATIO);
}

bool power_mgr_is_battery_low(void) {
  uint16_t mv = power_mgr_get_battery_mv();
  return (mv > 0 && mv < BAT_LOW_THRESHOLD_MV);
}

/* ---------- LCD Power ---------- */

esp_err_t power_mgr_set_lcd_power(bool enable) {
  gpio_set_level(LCD_BK_PIN, enable ? 1 : 0);
  ESP_LOGI(TAG, "LCD backlight %s (GPIO %d)", enable ? "ON" : "OFF",
           LCD_BK_PIN);
  return ESP_OK;
}

/* ---------- Sleep ---------- */

esp_err_t power_mgr_enter_sleep(uint32_t duration_ms) {
  /* Configure timer wakeup */
  esp_sleep_enable_timer_wakeup(duration_ms * 1000ULL);

  /* Configure GPIO wakeup for buttons (active low) */
  esp_sleep_enable_gpio_wakeup();
  gpio_wakeup_enable(BTN_A_PIN, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable(BTN_B_PIN, GPIO_INTR_LOW_LEVEL);

  /*
   * CRITICAL: Ensure POWER HOLD (GPIO 4) stays HIGH during light sleep.
   * esp_sleep_enable_gpio_wakeup() and the default pad state should handle this
   * for light sleep.
   */

  ESP_LOGD(TAG, "Entering light sleep (%lums)...", (unsigned long)duration_ms);
  esp_light_sleep_start();
  ESP_LOGD(TAG, "Woke up from sleep");

  return ESP_OK;
}
