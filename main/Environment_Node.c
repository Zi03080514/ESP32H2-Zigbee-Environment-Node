#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "bme280.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "esp_zigbee_core.h"
#include "nvs_flash.h"

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
static esp_err_t bme280_forced_mode_init(void);
static void bme280_init(void);
static void battery_adc_init(void);
static void nvs_init(void);
static void bme280_read(float *temp, float *hum, float *pres);
static void battery_adc_read(int *voltage_mv);


void app_main(void)
{
    ESP_LOGI(TAG, "System starting...");

    nvs_init();
    i2c_bus_init();
    bme280_init();
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
 * @brief Initialize BME280 Sensor (Forced Mode)
 * Reuse core steps from bme280_default_init but switch to Forced Mode.
 */
static esp_err_t bme280_forced_mode_init(void) {
    bme280_sensor = bme280_create(i2c_bus, BME280_I2C_ADDRESS_DEFAULT); // Address 0x76
    if (bme280_sensor == NULL) {
        return ESP_FAIL;
    }

    // check if sensor, i.e. the chip ID is correct
    uint8_t chipid = 0;
    bme280_dev_t *sens = (bme280_dev_t *) bme280_sensor;
    if (i2c_bus_read_byte(sens->i2c_dev, BME280_REGISTER_CHIPID, &chipid) != ESP_OK) {
        ESP_LOGI("BME280:", "bme280_default_init->bme280_read_byte ->BME280_REGISTER_CHIPID failed!!!!:%x", chipid);
        return ESP_FAIL;
    }
    if (chipid != BME280_DEFAULT_CHIPID) {
        ESP_LOGI("BME280:", "bme280_default_init->BME280_DEFAULT_CHIPID:%x", chipid);
        return ESP_FAIL;
    }
    // reset the sens using soft-reset, this makes sure the IIR is off, etc.
    if (i2c_bus_write_byte(sens->i2c_dev, BME280_REGISTER_SOFTRESET, 0xB6) != ESP_OK) {
        return ESP_FAIL;
    }
    // wait for chip to wake up.
    vTaskDelay(300 / portTICK_RATE_MS);
    // if chip is still reading calibration, delay
    while (bme280_is_reading_calibration(bme280_sensor)) {
        vTaskDelay(100 / portTICK_RATE_MS);
    }
    if (bme280_read_coefficients(bme280_sensor) != ESP_OK) { // read trimming parameters, see DS 4.2.2
        return ESP_FAIL;
    }
    // Apply configuration with Forced Mode
    if (bme280_set_sampling(bme280_sensor,
                            BME280_MODE_FORCED,     // Change: Normal -> Forced
                            BME280_SAMPLING_X16,
                            BME280_SAMPLING_X16,
                            BME280_SAMPLING_X16,
                            BME280_FILTER_OFF,
                            BME280_STANDBY_MS_0_5) != ESP_OK) {
        return ESP_FAIL;
                            }
    return ESP_OK;
}

/**
 * @brief Initialize BME280 Sensor (Forced Mode) with error handling
 * (Specifically used in this project)
 */
static void bme280_init(void) {
    ESP_ERROR_CHECK(bme280_forced_mode_init());
    ESP_LOGI(TAG, "BME280 initialized.");
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
 * @brief Initialize Non-Volatile Storage (NVS)
 * (Required by Zigbee stack)
 */
static void nvs_init(void)
{
    esp_err_t ret = nvs_flash_init();

    // If NVS partition is full or has a version mismatch, erase and re-init
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition is full or corrupted, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    // Check for other initialization errors
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized successfully.");
}

/**
 * @brief Read Data from BME280 Sensor
 */
static void bme280_read(float *temp, float *hum, float *pres) {
    ESP_ERROR_CHECK(bme280_take_forced_measurement(bme280_sensor));

    vTaskDelay(pdMS_TO_TICKS(150)); // Wait for measurement conversion

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
