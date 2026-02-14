/*
 * NexSense M5-DoorSensor — Sensor Module (MPU6886 IMU)
 *
 * MPU6886 I2C driver, accelerometer sampling, vector normalization,
 * and calibration persistence via NVS.
 *
 * Hardware: M5StickC Plus 2
 *   - MPU6886 on I2C0: SDA=GPIO21, SCL=GPIO22
 *   - I2C address: 0x68
 */

#include "sensor.h"

#include <math.h>
#include <string.h>

#include "driver/i2c.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"

/* ---------- Constants ---------- */

static const char *TAG = "sensor";

/* I2C configuration */
#define I2C_PORT I2C_NUM_0
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_FREQ_HZ 400000

/* MPU6886 registers */
#define MPU6886_ADDR 0x68
#define MPU6886_WHO_AM_I 0x75
#define MPU6886_PWR_MGMT_1 0x6B
#define MPU6886_PWR_MGMT_2 0x6C
#define MPU6886_SMPLRT_DIV 0x19
#define MPU6886_CONFIG 0x1A
#define MPU6886_ACCEL_CONFIG 0x1C
#define MPU6886_ACCEL_CONFIG2 0x1D
#define MPU6886_ACCEL_XOUT_H 0x3B
#define MPU6886_GYRO_CONFIG 0x1B
#define MPU6886_GYRO_XOUT_H 0x43
#define MPU6886_WHO_AM_I_VAL 0x19

/* Accel full-scale ±2g → 16384 LSB/g */
#define ACCEL_SENSITIVITY 16384.0f
#define GYRO_SENSITIVITY 16.4f /* ±2000dps */

/* Calibration sampling */
#define CAL_NUM_SAMPLES 32
#define CAL_SAMPLE_DELAY_MS 20

/* NVS namespace and keys */
#define NVS_NAMESPACE "doorsensor"
#define NVS_KEY_CAL "cal_data"

/* ---------- RTC Memory ---------- */

static RTC_DATA_ATTR calibration_t s_rtc_cal;

/* ---------- I2C Helpers ---------- */

static esp_err_t mpu_write_byte(uint8_t reg, uint8_t val) {
  uint8_t buf[2] = {reg, val};
  return i2c_master_write_to_device(I2C_PORT, MPU6886_ADDR, buf, 2,
                                    pdMS_TO_TICKS(100));
}

static esp_err_t mpu_read_bytes(uint8_t reg, uint8_t *data, size_t len) {
  return i2c_master_write_read_device(I2C_PORT, MPU6886_ADDR, &reg, 1, data,
                                      len, pdMS_TO_TICKS(100));
}

/* ---------- Initialization ---------- */

esp_err_t sensor_init(void) {
  esp_err_t ret;

  /* Configure I2C master */
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_SDA_PIN,
      .scl_io_num = I2C_SCL_PIN,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master =
          {
              .clk_speed = I2C_FREQ_HZ,
          },
      .clk_flags = 0,
  };
  ret = i2c_param_config(I2C_PORT, &conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
    return ret;
  }
  ret = i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
    return ret;
  }

  /* Verify MPU6886 identity */
  uint8_t who = 0;
  ret = mpu_read_bytes(MPU6886_WHO_AM_I, &who, 1);
  if (ret != ESP_OK || who != MPU6886_WHO_AM_I_VAL) {
    ESP_LOGE(TAG, "MPU6886 not found (WHO_AM_I=0x%02X, expected 0x%02X)", who,
             MPU6886_WHO_AM_I_VAL);
    return ESP_ERR_NOT_FOUND;
  }
  ESP_LOGI(TAG, "MPU6886 detected (WHO_AM_I=0x%02X)", who);

  /* Reset device */
  mpu_write_byte(MPU6886_PWR_MGMT_1, 0x80); /* Device reset */
  vTaskDelay(pdMS_TO_TICKS(100));

  /* Wake up, auto-select clock */
  mpu_write_byte(MPU6886_PWR_MGMT_1, 0x01); /* Clock = PLL */
  vTaskDelay(pdMS_TO_TICKS(10));

  /* Enable accel only, disable gyro to save power */
  mpu_write_byte(MPU6886_PWR_MGMT_2, 0x07); /* Gyro X/Y/Z disabled */

  /* Accel config: ±2g full scale */
  mpu_write_byte(MPU6886_ACCEL_CONFIG, 0x00);

  /* Accel LPF: bandwidth 218 Hz */
  mpu_write_byte(MPU6886_ACCEL_CONFIG2, 0x01);

  /* Gyro config: ±2000dps full scale */
  mpu_write_byte(MPU6886_GYRO_CONFIG, 0x18);

  /* Sample rate divider: 1kHz / (1+9) = 100 Hz */
  mpu_write_byte(MPU6886_SMPLRT_DIV, 0x09);

  ESP_LOGI(TAG, "MPU6886 initialized (±2g, 100Hz, gyro disabled)");
  return ESP_OK;
}

/* ---------- Reading ---------- */

esp_err_t sensor_read_accel(float xyz[3]) {
  uint8_t raw[6];
  esp_err_t ret = mpu_read_bytes(MPU6886_ACCEL_XOUT_H, raw, 6);
  if (ret != ESP_OK) {
    return ret;
  }

  /* Convert big-endian raw to signed 16-bit, then to g */
  int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
  int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
  int16_t az = (int16_t)((raw[4] << 8) | raw[5]);

  xyz[0] = (float)ax / ACCEL_SENSITIVITY;
  xyz[1] = (float)ay / ACCEL_SENSITIVITY;
  xyz[2] = (float)az / ACCEL_SENSITIVITY;

  return ESP_OK;
}

void sensor_normalize(float xyz[3]) {
  float mag = sqrtf(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2]);
  if (mag > 0.001f) {
    xyz[0] /= mag;
    xyz[1] /= mag;
    xyz[2] /= mag;
  }
}

/* ---------- Calibration Persistence ---------- */

esp_err_t sensor_load_calibration(calibration_t *cal) {
  nvs_handle_t nvs;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
  if (ret != ESP_OK) {
    return ret;
  }

  size_t len = sizeof(calibration_t);
  ret = nvs_get_blob(nvs, NVS_KEY_CAL, cal, &len);
  nvs_close(nvs);

  if (ret == ESP_OK && cal->valid) {
    /* Cache in RTC memory for fast access after sleep */
    memcpy(&s_rtc_cal, cal, sizeof(calibration_t));
    ESP_LOGI(TAG, "Calibration loaded from NVS (calibrated_at=%lu)",
             (unsigned long)cal->calibrated_at);
  }

  return ret;
}

esp_err_t sensor_save_calibration(const calibration_t *cal) {
  nvs_handle_t nvs;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = nvs_set_blob(nvs, NVS_KEY_CAL, cal, sizeof(calibration_t));
  if (ret == ESP_OK) {
    ret = nvs_commit(nvs);
  }
  nvs_close(nvs);

  if (ret == ESP_OK) {
    /* Update RTC cache */
    memcpy(&s_rtc_cal, cal, sizeof(calibration_t));
    ESP_LOGI(TAG, "Calibration saved to NVS");
  }

  return ret;
}

/* ---------- Calibration Capture ---------- */

static esp_err_t calibrate_average(float ref[3]) {
  float sum[3] = {0.0f, 0.0f, 0.0f};

  for (int i = 0; i < CAL_NUM_SAMPLES; i++) {
    float sample[3];
    esp_err_t ret = sensor_read_accel(sample);
    if (ret != ESP_OK) {
      return ret;
    }
    sum[0] += sample[0];
    sum[1] += sample[1];
    sum[2] += sample[2];
    vTaskDelay(pdMS_TO_TICKS(CAL_SAMPLE_DELAY_MS));
  }

  ref[0] = sum[0] / CAL_NUM_SAMPLES;
  ref[1] = sum[1] / CAL_NUM_SAMPLES;
  ref[2] = sum[2] / CAL_NUM_SAMPLES;

  /* Normalize */
  sensor_normalize(ref);

  ESP_LOGI(TAG, "Calibration captured: [%.3f, %.3f, %.3f]", ref[0], ref[1],
           ref[2]);
  return ESP_OK;
}

esp_err_t sensor_calibrate_open(calibration_t *cal) {
  ESP_LOGI(TAG, "Calibrating OPEN position...");
  return calibrate_average(cal->open_ref);
}

esp_err_t sensor_calibrate_closed(calibration_t *cal) {
  ESP_LOGI(TAG, "Calibrating CLOSED position...");
  esp_err_t ret = calibrate_average(cal->closed_ref);
  if (ret == ESP_OK) {
    cal->calibrated_at = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    cal->valid = true;
  }
  return ret;
}

esp_err_t sensor_read_gyro(float xyz[3]) {
  uint8_t raw[6];
  esp_err_t ret = mpu_read_bytes(MPU6886_GYRO_XOUT_H, raw, 6);
  if (ret != ESP_OK) {
    return ret;
  }

  /* Convert big-endian raw to signed 16-bit, then to dps */
  int16_t gx = (int16_t)((raw[0] << 8) | raw[1]);
  int16_t gy = (int16_t)((raw[2] << 8) | raw[3]);
  int16_t gz = (int16_t)((raw[4] << 8) | raw[5]);

  xyz[0] = (float)gx / GYRO_SENSITIVITY;
  xyz[1] = (float)gy / GYRO_SENSITIVITY;
  xyz[2] = (float)gz / GYRO_SENSITIVITY;

  return ESP_OK;
}

esp_err_t sensor_enable_gyro(bool enable) {
  /*
   * PWR_MGMT_2 bits:
   * [5:3] Accel standby, [2:0] Gyro standby
   * 0x00: Both enabled
   * 0x07: Accel enabled, Gyro standby
   */
  uint8_t val = enable ? 0x00 : 0x07;
  esp_err_t ret = mpu_write_byte(MPU6886_PWR_MGMT_2, val);
  vTaskDelay(pdMS_TO_TICKS(10));
  return ret;
}

esp_err_t sensor_set_low_power(bool enable) {
  /*
   * For MPU6886, we can reduce the sample rate in regular mode or
   * use the "Low Power Accel" cycle mode. For simplicity, we just
   * change the sample rate divider.
   * 100Hz: div=9 (1000 / (1+9))
   * 10Hz:  div=99 (1000 / (1+99))
   */
  uint8_t div = enable ? 99 : 9;
  return mpu_write_byte(MPU6886_SMPLRT_DIV, div);
}

const calibration_t *sensor_get_calibration(void) { return &s_rtc_cal; }
