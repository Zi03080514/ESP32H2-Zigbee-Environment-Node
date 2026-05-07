#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "bme280.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_zigbee_core.h"
#include "zcl/esp_zigbee_zcl_power_config.h"
#include "nvs_flash.h"

// Zigbee device info
#define ZB_ENDPOINT 1
#define ZB_MANUFACTURER_NAME "\x0C""Thingsmaking"
#define ZB_MODEL_IDENTIFIER "\x07""TML-EN1"

// Zigbee reporting logic configuration
#define ZB_REPORT_MIN_INTERVAL      10 // 10 seconds
#define ZB_REPORT_MAX_INTERVAL      (10 * 60) // 10 minutes
#define SENSOR_CHECK_INTERVAL_MS       (60 * 1000)      // 1 minute
static int16_t  report_temp_threshold  = 50; // 0.5C
static uint16_t report_hum_threshold   = 200; // 2%
static int16_t  report_press_threshold = 1; // 0.1 kPa
static uint8_t  report_batt_threshold  = 2; // 1%

// I2C configuration
#define I2C_MASTER_SDA_IO 10
#define I2C_MASTER_SCL_IO 11
#define I2C_MASTER_NUM    I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

// ADC configuration
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_0 // GPIO 1

// Define the tag for logs
static const char *TAG = "TML_EN1";

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
static void bme280_read(float *temp, float *hum, float *press);
static void battery_adc_read(int *voltage_mv);
uint8_t calc_battery_percentage(uint32_t voltage_mv);

static void zb_configure_reporting(void);
static void esp_zb_task(void *pvParameter);
static void sensor_task(void *pvParameters);
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct);
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message);


void app_main(void)
{
    ESP_LOGI(TAG, "System starting...");

    // Configure power management
    esp_pm_config_t pm_config = {
        .max_freq_mhz = 96,
        .min_freq_mhz = 32,
        .light_sleep_enable = true,
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));

    // Initialize hardware
    nvs_init();
    i2c_bus_init();
    bme280_init();
    battery_adc_init();

    // Create Zigbee stack task
    xTaskCreate(esp_zb_task, "Zigbee_task", 4096, NULL, 5, NULL);

    // Create sensor task
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 3, NULL);

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
static void bme280_read(float *temp, float *hum, float *press) {
    ESP_ERROR_CHECK(bme280_take_forced_measurement(bme280_sensor));

    vTaskDelay(pdMS_TO_TICKS(150)); // Wait for measurement conversion

    bme280_read_temperature(bme280_sensor, temp);
    bme280_read_humidity(bme280_sensor, hum);
    bme280_read_pressure(bme280_sensor, press);
}

/**
 * @brief Read Data from ADC
 */
static void battery_adc_read(int *voltage_mv) {
    int adc_raw;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BATTERY_ADC_CHANNEL, &adc_raw));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, voltage_mv));
}

/**
 * @brief Calculate Battery Percentage (0-100)
 */
uint8_t calc_battery_percentage(uint32_t voltage_mv) {
    // Discharge curve thresholds normalized to 0-100
    static const struct {
        uint16_t mv;
        uint8_t pct;
    } curve[] = {
        {3200, 0},   {3400, 10},  {3600, 20},
        {3700, 40},  {3800, 60},  {3900, 80},
        {4000, 90},  {4200, 100}
    };

    const int num_points = sizeof(curve) / sizeof(curve[0]);

    // Handle out-of-bounds
    if (voltage_mv <= curve[0].mv) {
        return 0;
    }
    if (voltage_mv >= curve[num_points - 1].mv) {
        return 100;
    }

    // Linear interpolation
    for (int i = 0; i < num_points - 1; i++) {
        if (voltage_mv >= curve[i].mv && voltage_mv <= curve[i + 1].mv) {
            uint32_t v_range = curve[i + 1].mv - curve[i].mv;
            uint32_t p_range = curve[i + 1].pct - curve[i].pct;

            return curve[i].pct +
                   ((voltage_mv - curve[i].mv) * p_range) / v_range;
        }
    }

    return 0;
}

/**
 * @brief Configure Automatic Reporting for all sensors
 */
static void zb_configure_reporting(void) {
    esp_zb_zcl_config_report_cmd_t report_cmd = {0};
    report_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    report_cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000; // Destination: Coordinator
    report_cmd.zcl_basic_cmd.src_endpoint = ZB_ENDPOINT;
    report_cmd.zcl_basic_cmd.dst_endpoint = 1;

    // Temperature Reporting
    report_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
    esp_zb_zcl_config_report_record_t temp_record = {
        .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
        .attributeID = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        .attrType = ESP_ZB_ZCL_ATTR_TYPE_S16,
        .min_interval = ZB_REPORT_MIN_INTERVAL,
        .max_interval = ZB_REPORT_MAX_INTERVAL,
        .reportable_change = &report_temp_threshold,
    };
    report_cmd.record_number = 1;
    report_cmd.record_field = &temp_record;
    esp_zb_zcl_config_report_cmd_req(&report_cmd);

    // Humidity Reporting
    report_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;
    esp_zb_zcl_config_report_record_t hum_record = {
        .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
        .attributeID = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
        .attrType = ESP_ZB_ZCL_ATTR_TYPE_U16,
        .min_interval = ZB_REPORT_MIN_INTERVAL,
        .max_interval = ZB_REPORT_MAX_INTERVAL,
        .reportable_change = &report_hum_threshold,
    };
    report_cmd.record_field = &hum_record;
    esp_zb_zcl_config_report_cmd_req(&report_cmd);

    // Pressure Reporting
    report_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT;
    esp_zb_zcl_config_report_record_t press_record = {
        .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
        .attributeID = ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID,
        .attrType = ESP_ZB_ZCL_ATTR_TYPE_S16,
        .min_interval = ZB_REPORT_MIN_INTERVAL,
        .max_interval = ZB_REPORT_MAX_INTERVAL,
        .reportable_change = &report_press_threshold,
    };
    report_cmd.record_field = &press_record;
    esp_zb_zcl_config_report_cmd_req(&report_cmd);

    // Battery Reporting
    report_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG;
    esp_zb_zcl_config_report_record_t batt_record = {
        .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
        .attributeID = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
        .attrType = ESP_ZB_ZCL_ATTR_TYPE_U8,
        .min_interval = ZB_REPORT_MIN_INTERVAL,
        .max_interval = ZB_REPORT_MAX_INTERVAL,
        .reportable_change = &report_batt_threshold,
    };
    report_cmd.record_field = &batt_record;
    esp_zb_zcl_config_report_cmd_req(&report_cmd);

    ESP_LOGI(TAG, "Automatic reporting configured.");
}

/**
 * @brief Zigbee Stack Task
 */
static void esp_zb_task(void *pvParameter) {
    // Initialize Zigbee stack
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,
        .install_code_policy = false,
        .nwk_cfg.zed_cfg = {
            .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN,
            .keep_alive = 60000,
        },
    };
    esp_zb_init(&zb_nwk_cfg);

    // Configure Zigbee sleep
    esp_zb_set_rx_on_when_idle(false);
    esp_zb_sleep_enable(true);
    esp_zb_sleep_set_threshold(2000);

    // Create cluster list
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    // Create basic cluster
    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_BATTERY,
    };

    esp_zb_attribute_list_t *basic_attr_list = esp_zb_basic_cluster_create(&basic_cfg);

    esp_zb_basic_cluster_add_attr(basic_attr_list, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ZB_MANUFACTURER_NAME);
    esp_zb_basic_cluster_add_attr(basic_attr_list, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ZB_MODEL_IDENTIFIER);

    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Create temperature sensor cluster
    esp_zb_temperature_meas_cluster_cfg_t temp_cfg = {
        .min_value = -1000, // -10.00C
        .max_value = 5000,  // 50.00C
    };
    esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list,
        esp_zb_temperature_meas_cluster_create(&temp_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Create humidity sensor cluster
    esp_zb_humidity_meas_cluster_cfg_t hum_cfg = {
        .min_value = 0,    // 0%
        .max_value = 10000 // 100.00%
    };
    esp_zb_cluster_list_add_humidity_meas_cluster(cluster_list,
    esp_zb_humidity_meas_cluster_create(&hum_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Create pressure sensor cluster
    esp_zb_pressure_meas_cluster_cfg_t press_cfg = {
        .min_value = 300,  // 30.0 kPa
        .max_value = 1100, // 110.0 hPa
    };
    esp_zb_cluster_list_add_pressure_meas_cluster(cluster_list,
        esp_zb_pressure_meas_cluster_create(&press_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Create power cluster
    esp_zb_attribute_list_t *power_cfg_cluster = esp_zb_power_config_cluster_create(NULL);
    uint8_t battery_percentage = 200; // Initialize with 100%
    esp_zb_power_config_cluster_add_attr(power_cfg_cluster,
                            ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
                            &battery_percentage);
    esp_zb_cluster_list_add_power_config_cluster(cluster_list, power_cfg_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Create endpoint
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t ep_config = {
        .endpoint = ZB_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, cluster_list, ep_config);

    // Register the action handler
    esp_zb_core_action_handler_register(zb_action_handler);

    // Start Zigbee stack
    esp_zb_device_register(ep_list);
    esp_zb_set_primary_network_channel_set(1l << 11); // Channel 11
    ESP_ERROR_CHECK(esp_zb_start(false));

    esp_zb_stack_main_loop();

}

/**
 * @brief Sensor & Logic Task
 */
static void sensor_task(void *pvParameters) {
    while (1) {
        float temp, hum, press;
        int battery_voltage;

        bme280_read(&temp, &hum, &press);
        battery_adc_read(&battery_voltage);
        battery_voltage *= 2; // Restore to the actual battery voltage
        uint8_t battery_percentage = calc_battery_percentage(battery_voltage);
        ESP_LOGI(TAG, "Readings: Temp: %.2f C, Humidity: %.2f %%, Pressure: %.2f kPa, Bat: %d %%", temp, hum, press/10, battery_percentage);

        // Update Zigbee attributes.
        // Update Temperature
        int16_t zcl_temp = (int16_t)(temp * 100);
        esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
                                     &zcl_temp, false);

        // Update Humidity
        uint16_t zcl_hum = (uint16_t)(hum * 100);
        esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
                                     &zcl_hum, false);

        // Update Pressure
        int16_t zcl_press = (int16_t)press;
        esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID,
                                     &zcl_press, false);

        // Update Battery (Simple 10-step logic)
        uint8_t zcl_batt = battery_percentage * 2;
        esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
                                     &zcl_batt, false);

        vTaskDelay(pdMS_TO_TICKS(SENSOR_CHECK_INTERVAL_MS));

    }
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
    esp_err_t err = esp_zb_bdb_start_top_level_commissioning(mode_mask);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start Zigbee commissioning via alarm, error: 0x%x", err);
    }
}

/**
 * @brief Zigbee Signal Handler
 */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            ESP_LOGI(TAG, "Initialize Zigbee stack");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
            break;
        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (err_status == ESP_OK) {
                if (esp_zb_bdb_is_factory_new()) {
                    ESP_LOGI(TAG, "Device is factory new, start network steering");
                    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                } else {
                    ESP_LOGI(TAG, "Device rebooted, already in network.");
                    zb_configure_reporting();
                }
            }
            break;
        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err_status == ESP_OK) {
                ESP_LOGI(TAG, "Joined network successfully");
                zb_configure_reporting();
            } else {
                ESP_LOGW(TAG, "Network steering failed, retrying in 1s");
                esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            }
            break;
        case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
            // Handle Zigbee sleep
            ESP_LOGI(TAG, "Zigbee stack indicates it can sleep now");
            esp_zb_sleep_now();
            break;
        default:
            break;
    }
}

/**
 * @brief Zigbee Action Handler
 */
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message) {
    esp_err_t ret = ESP_OK;

    switch (callback_id) {
        case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
            ESP_LOGI(TAG, "Receive ZCL set attribute request");
            break;
        case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
            ESP_LOGI(TAG, "Receive ZCL command default response");
            break;
        default:
            ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
            break;
    }
    return ret;
}