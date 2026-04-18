#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "bme280.h"
#include "esp_log.h"

// Define the tag for logs
static const char *TAG = "TML_EN1";

// I2C configuration
#define I2C_MASTER_SDA_IO 10
#define I2C_MASTER_SCL_IO 11
#define I2C_MASTER_NUM    I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

// BME280 & I2C Bus handle
static bme280_handle_t sensor = NULL;
static i2c_bus_handle_t i2c_bus = NULL;

void app_main(void)
{
    ESP_LOGI(TAG, "System starting...");

    // Setup I2C Bus
    ESP_LOGI(TAG, "Initializing I2C Bus...");
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &conf);

    // Create BME280 Handle
    ESP_LOGI(TAG, "Initializing BME280 Sensor...");
    sensor = bme280_create(i2c_bus, BME280_I2C_ADDRESS_DEFAULT); // Address 0x76
    if (sensor == NULL) {
        ESP_LOGE(TAG, "Failed to create BME280 handle!");
        return;
    }

    // Initialize the sensor
    if (bme280_default_init(sensor) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BME280!");
        return;
    }
    ESP_LOGI(TAG, "BME280 initialized successfully.");

    float temperature, humidity, pressure;

    while (1) {
        // Read data from the sensor
        bme280_read_temperature(sensor, &temperature);
        bme280_read_humidity(sensor, &humidity);
        bme280_read_pressure(sensor, &pressure);

        ESP_LOGI(TAG, "Readings: Temp: %.2f C, Humidity: %.2f %%, Pressure: %.2f hPa", temperature, humidity, pressure);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }

}
