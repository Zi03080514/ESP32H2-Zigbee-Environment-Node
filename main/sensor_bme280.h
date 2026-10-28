#pragma once

#include "esp_err.h"
#include "i2c_bus.h"

#if __has_include("config.h")
#include "config.h"
#else
#include "config.h.example"
#endif

#if ENABLE_BME280

/**
 * @brief Initialize BME280 sensor in Forced Mode
 *
 * @param bus I2C bus handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sensor_bme280_init(i2c_bus_handle_t bus);

/**
 * @brief Trigger a single forced measurement
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sensor_bme280_trigger(void);

/**
 * @brief Read temperature, humidity and pressure data
 *
 * @param[out] temp Temperature in degrees Celsius
 * @param[out] hum Relative humidity in percentage
 * @param[out] press Atmospheric pressure in hPa
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sensor_bme280_read(float *temp, float *hum, float *press);

#endif // ENABLE_BME280

