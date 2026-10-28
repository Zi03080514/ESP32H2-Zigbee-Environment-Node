#include "sensor_bme280.h"

#if ENABLE_BME280

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bme280.h"

static const char *TAG = "SENSOR_BME280";
static bme280_handle_t bme280_sensor = NULL;

esp_err_t sensor_bme280_init(i2c_bus_handle_t bus) {
    if (!bus) {
        return ESP_ERR_INVALID_ARG;
    }

    bme280_sensor = bme280_create(bus, BME280_I2C_ADDR);
    if (bme280_sensor == NULL) {
        ESP_LOGW(TAG, "Failed to create BME280 device.");
        return ESP_FAIL;
    }

    uint8_t chipid = 0;
    bme280_dev_t *sens = (bme280_dev_t *)bme280_sensor;
    if (i2c_bus_read_byte(sens->i2c_dev, BME280_REGISTER_CHIPID, &chipid) != ESP_OK) {
        ESP_LOGW(TAG, "Reading CHIPID failed: 0x%02x", chipid);
        return ESP_FAIL;
    }
    if (chipid != BME280_DEFAULT_CHIPID) {
        ESP_LOGW(TAG, "Unexpected CHIPID: 0x%02x", chipid);
        return ESP_FAIL;
    }

    if (i2c_bus_write_byte(sens->i2c_dev, BME280_REGISTER_SOFTRESET, 0xB6) != ESP_OK) {
        return ESP_FAIL;
    }

    vTaskDelay(pdMS_TO_TICKS(300));
    while (bme280_is_reading_calibration(bme280_sensor)) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (bme280_read_coefficients(bme280_sensor) != ESP_OK) {
        return ESP_FAIL;
    }

    if (bme280_set_sampling(bme280_sensor,
                            BME280_MODE_FORCED,
                            BME280_SAMPLING_X16,
                            BME280_SAMPLING_X16,
                            BME280_SAMPLING_X16,
                            BME280_FILTER_OFF,
                            BME280_STANDBY_MS_0_5) != ESP_OK) {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "BME280 initialized successfully.");
    return ESP_OK;
}

esp_err_t sensor_bme280_trigger(void) {
    if (!bme280_sensor) {
        return ESP_ERR_INVALID_STATE;
    }
    return bme280_take_forced_measurement(bme280_sensor);
}

esp_err_t sensor_bme280_read(float *temp, float *hum, float *press) {
    if (!bme280_sensor || !temp || !hum || !press) {
        return ESP_ERR_INVALID_ARG;
    }
    if (bme280_read_temperature(bme280_sensor, temp) != ESP_OK ||
        bme280_read_humidity(bme280_sensor, hum) != ESP_OK ||
        bme280_read_pressure(bme280_sensor, press) != ESP_OK) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

#endif // ENABLE_BME280

