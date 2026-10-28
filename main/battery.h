#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize battery ADC channel and calibration scheme
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t battery_init(void);

/**
 * @brief Read battery voltage and calculate remaining percentage
 *
 * @param[out] voltage_mv Pointer to store measured voltage in millivolts (optional, can be NULL)
 * @param[out] percentage Pointer to store battery percentage [0-100] (optional, can be NULL)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t battery_read(int *voltage_mv, uint8_t *percentage);

#ifdef __cplusplus
}
#endif

