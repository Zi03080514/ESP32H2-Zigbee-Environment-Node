#pragma once

#include "esp_err.h"
#include "i2c_bus.h"

#if __has_include("config.h")
#include "config.h"
#else
#include "config.h.example"
#endif

#if ENABLE_BH1750

/**
 * @brief Initialize BH1750 sensor handle
 *
 * @param bus I2C bus handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sensor_bh1750_init(i2c_bus_handle_t bus);

/**
 * @brief Trigger One-Time High-Resolution measurement (auto power-down after conversion)
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sensor_bh1750_trigger(void);

/**
 * @brief Read measured ambient light level in Lux
 *
 * @param[out] lux Light level in Lux
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sensor_bh1750_read(float *lux);

#endif // ENABLE_BH1750

