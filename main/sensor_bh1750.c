#include "sensor_bh1750.h"

#if ENABLE_BH1750

#include "esp_log.h"

#define BH1750_CMD_ONE_TIME_H_RES 0x20

static const char *TAG = "SENSOR_BH1750";
static i2c_bus_device_handle_t bh1750_dev = NULL;

esp_err_t sensor_bh1750_init(i2c_bus_handle_t bus) {
    if (!bus) {
        return ESP_ERR_INVALID_ARG;
    }
    bh1750_dev = i2c_bus_device_create(bus, BH1750_I2C_ADDR, I2C_MASTER_FREQ_HZ);
    if (!bh1750_dev) {
        ESP_LOGW(TAG, "Failed to create BH1750 device.");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BH1750 initialized successfully.");
    return ESP_OK;
}

esp_err_t sensor_bh1750_trigger(void) {
    if (!bh1750_dev) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_bus_write_byte(bh1750_dev, NULL_I2C_MEM_ADDR, BH1750_CMD_ONE_TIME_H_RES);
}

esp_err_t sensor_bh1750_read(float *lux) {
    if (!bh1750_dev || !lux) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t data[2] = {0};
    esp_err_t ret = i2c_bus_read_bytes(bh1750_dev, NULL_I2C_MEM_ADDR, 2, data);
    if (ret != ESP_OK) {
        return ret;
    }
    uint16_t raw = ((uint16_t)data[0] << 8) | data[1];
    *lux = (float)raw / 1.2f;
    return ESP_OK;
}

#endif // ENABLE_BH1750

