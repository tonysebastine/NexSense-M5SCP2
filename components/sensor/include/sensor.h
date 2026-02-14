/*
 * NexSense M5-DoorSensor — Sensor Module (MPU6886 IMU)
 *
 * Provides accelerometer sampling, vector normalization,
 * and calibration model with NVS persistence.
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Types ---------- */

typedef struct {
  float open_ref[3];      /* Normalized reference vector for OPEN position  */
  float closed_ref[3];    /* Normalized reference vector for CLOSED position */
  uint32_t calibrated_at; /* Timestamp (seconds since boot) of calibration   */
  bool valid;             /* True if calibration has been performed           */
} calibration_t;

/* ---------- Initialization ---------- */

/**
 * @brief Initialize the MPU6886 IMU over I2C.
 * @return ESP_OK on success.
 */
esp_err_t sensor_init(void);

/* ---------- Reading ---------- */

/**
 * @brief Enable or disable the gyroscope.
 * @param[in] enable  true to enable, false to disable.
 * @return ESP_OK on success.
 */
esp_err_t sensor_enable_gyro(bool enable);

/**
 * @brief Read raw gyroscope values in degrees per second (dps).
 * @param[out] xyz  Array of 3 floats: [x, y, z] in dps.
 * @return ESP_OK on success.
 */
esp_err_t sensor_read_gyro(float xyz[3]);

/**
 * @brief Read raw accelerometer values in g.
 * @param[out] xyz  Array of 3 floats: [x, y, z] in g.
 * @return ESP_OK on success.
 */
esp_err_t sensor_read_accel(float xyz[3]);

/**
 * @brief Normalize a 3-axis vector in-place to unit length.
 * @param[in,out] xyz  Array of 3 floats.
 */
void sensor_normalize(float xyz[3]);

/* ---------- Calibration ---------- */

/**
 * @brief Load calibration data from NVS.
 * @param[out] cal  Populated calibration struct.
 * @return ESP_OK if found, ESP_ERR_NVS_NOT_FOUND if not calibrated.
 */
esp_err_t sensor_load_calibration(calibration_t *cal);

/**
 * @brief Save calibration data to NVS.
 * @param[in] cal  Calibration struct to persist.
 * @return ESP_OK on success.
 */
esp_err_t sensor_save_calibration(const calibration_t *cal);

/**
 * @brief Capture current orientation as OPEN reference.
 * Averages multiple samples for stability.
 * @param[in,out] cal  Calibration struct — open_ref will be written.
 * @return ESP_OK on success.
 */
esp_err_t sensor_calibrate_open(calibration_t *cal);

/**
 * @brief Capture current orientation as CLOSED reference.
 * Averages multiple samples for stability.
 * @param[in,out] cal  Calibration struct — closed_ref will be written.
 * @return ESP_OK on success.
 */
esp_err_t sensor_calibrate_closed(calibration_t *cal);

/**
 * @brief Set sensor to low-power mode (reduce sample rate).
 * @param[in] enable  true to enable LP mode (10Hz), false for normal (100Hz).
 * @return ESP_OK on success.
 */
esp_err_t sensor_set_low_power(bool enable);

/**
 * @brief Get the pointer to the active calibration (in RTC memory).
 * @return Pointer to calibration_t in RTC slow memory.
 */
const calibration_t *sensor_get_calibration(void);

#ifdef __cplusplus
}
#endif
