#include "sensor_scd41.h"

#if ENABLE_SCD41

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SENSOR_SCD41";
static i2c_bus_device_handle_t scd41_dev = NULL;

static uint8_t scd41_calc_crc(const uint8_t *data, uint16_t count) {
    uint8_t crc = 0xFF;
    for (uint16_t i = 0; i < count; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

void sensor_scd41_wake_up(void) {
    if (!scd41_dev) return;
    const uint8_t cmd[2] = {0x36, 0xF6};
    // Note: SCD41 does not ACK wake_up command in sleep mode; ignore return value
    i2c_bus_write_bytes(scd41_dev, NULL_I2C_MEM_ADDR, 2, cmd);
    vTaskDelay(pdMS_TO_TICKS(25));
}

esp_err_t sensor_scd41_power_down(void) {
    if (!scd41_dev) return ESP_ERR_INVALID_STATE;
    const uint8_t cmd[2] = {0x36, 0xE0};
    return i2c_bus_write_bytes(scd41_dev, NULL_I2C_MEM_ADDR, 2, cmd);
}

esp_err_t sensor_scd41_init(i2c_bus_handle_t bus) {
    if (!bus) return ESP_ERR_INVALID_ARG;
    scd41_dev = i2c_bus_device_create(bus, SCD41_I2C_ADDR, I2C_MASTER_FREQ_HZ);
    if (!scd41_dev) {
        ESP_LOGW(TAG, "Failed to create SCD41 device.");
        return ESP_FAIL;
    }
    sensor_scd41_wake_up();
    sensor_scd41_power_down();
    ESP_LOGI(TAG, "SCD41 initialized successfully (set to power-down).");
    return ESP_OK;
}

esp_err_t sensor_scd41_trigger(void) {
    if (!scd41_dev) return ESP_ERR_INVALID_STATE;
    const uint8_t cmd[2] = {0x21, 0x9D};
    return i2c_bus_write_bytes(scd41_dev, NULL_I2C_MEM_ADDR, 2, cmd);
}

esp_err_t sensor_scd41_read(uint16_t *co2_ppm) {
    if (!scd41_dev || !co2_ppm) return ESP_ERR_INVALID_ARG;
    const uint8_t cmd[2] = {0xEC, 0x05};
    esp_err_t ret = i2c_bus_write_bytes(scd41_dev, NULL_I2C_MEM_ADDR, 2, cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
    uint8_t data[9] = {0};
    ret = i2c_bus_read_bytes(scd41_dev, NULL_I2C_MEM_ADDR, 9, data);
    if (ret != ESP_OK) {
        return ret;
    }
    if (scd41_calc_crc(data, 2) != data[2]) {
        ESP_LOGW(TAG, "SCD41 CO2 CRC mismatch!");
        return ESP_ERR_INVALID_CRC;
    }
    *co2_ppm = ((uint16_t)data[0] << 8) | data[1];
    return ESP_OK;
}

#endif // ENABLE_SCD41

