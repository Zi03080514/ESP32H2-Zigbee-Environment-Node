#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "bme280.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

// Define the tag for logs
static const char *TAG = "TML_EN1";

// I2C configuration
#define I2C_MASTER_SDA_IO 10
#define I2C_MASTER_SCL_IO 11
#define I2C_MASTER_NUM    I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

// ADC configuration
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_0 // GPIO 1

// Global handles
static bme280_handle_t bme280_sensor = NULL;
static i2c_bus_handle_t i2c_bus = NULL;
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle = NULL;

// Custom functions
static void i2c_bus_init(void);
static esp_err_t bme280_init(void);
static void battery_adc_init(void);
static void bme280_read(float *temp, float *hum, float *pres);
static void battery_adc_read(int *voltage_mv);


void app_main(void)
{
    ESP_LOGI(TAG, "System starting...");

    i2c_bus_init();

    if (bme280_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BME280!");
        return;
    }
    ESP_LOGI(TAG, "BME280 initialized successfully.");

    battery_adc_init();

    float temperature, humidity, pressure;
    int voltage;

    while (1) {
        // Read data from the sensor
        bme280_read(&temperature, &humidity, &pressure);
        battery_adc_read(&voltage);

        ESP_LOGI(TAG, "Readings: Temp: %.2f C, Humidity: %.2f %%, Pressure: %.2f hPa, Bat: %d", temperature, humidity, pressure, voltage);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }

}

/**
 * @brief Initialize I2C Bus
 */
static void i2c_bus_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &conf);
    ESP_LOGI(TAG, "I2C Bus initialized.");
}

/**
 * @brief Initialize BME280 Sensor
 */
static esp_err_t bme280_init(void) {
    bme280_sensor = bme280_create(i2c_bus, BME280_I2C_ADDRESS_DEFAULT); // Address 0x76
    if (bme280_sensor == NULL) {
        return ESP_FAIL;
    }
    return bme280_default_init(bme280_sensor);
}

/**
 * @brief Initialize ADC (with Calibration) for Battery Voltage Monitoring
 */
static void battery_adc_init(void) {
    // ADC Unit Setup
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // ADC Channel Setup
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12, // 12dB attenuation for ~3V range
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, BATTERY_ADC_CHANNEL, &config));

    // ADC Calibration Setup
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = BATTERY_ADC_CHANNEL,      // Important: Specific to the structure you found
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle));

    ESP_LOGI(TAG, "ADC initialized.");
}

/**
 * @brief Read Data from BME280 Sensor
 */
static void bme280_read(float *temp, float *hum, float *pres) {
    bme280_read_temperature(bme280_sensor, temp);
    bme280_read_humidity(bme280_sensor, hum);
    bme280_read_pressure(bme280_sensor, pres);
}

/**
 * @brief Read Data from ADC
 */
static void battery_adc_read(int *voltage_mv) {
    int adc_raw;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BATTERY_ADC_CHANNEL, &adc_raw));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, voltage_mv));
}
