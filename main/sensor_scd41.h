#pragma once

#include "esp_err.h"
#include "i2c_bus.h"

#if __has_include("config.h")
#include "config.h"
#else
#include "config.h.example"
#endif

#if ENABLE_SCD41

/**
 * @brief Initialize SCD41 sensor handle and ensure it enters sleep mode
 *
 * @param bus I2C bus handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sensor_scd41_init(i2c_bus_handle_t bus);

/**
 * @brief Wake up SCD41 from sleep into idle mode (~25ms wait)
 */
void sensor_scd41_wake_up(void);

/**
 * @brief Trigger on-demand single shot measurement (~5000ms duration)
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sensor_scd41_trigger(void);

/**
 * @brief Read measured CO2 concentration with CRC verification
 *
 * @param[out] co2_ppm CO2 level in parts per million (ppm)
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_CRC if CRC check fails
 */
esp_err_t sensor_scd41_read(uint16_t *co2_ppm);

/**
 * @brief Put SCD41 into low-power sleep mode (<1 uA)
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sensor_scd41_power_down(void);

#endif // ENABLE_SCD41

