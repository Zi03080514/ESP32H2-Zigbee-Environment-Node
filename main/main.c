#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_zigbee_core.h"
#include "zcl/esp_zigbee_zcl_power_config.h"
#include "nvs_flash.h"
#include "esp_timer.h"

#if __has_include("config.h")
#include "config.h"
#else
#include "config.h.example"
#endif

#include "sensor_bme280.h"
#include "sensor_bh1750.h"
#include "sensor_scd41.h"
#include "battery.h"

// Define the tag for logs
static const char *TAG = "TML_EN1";

static uint32_t get_base_interval_s(void) {
    uint32_t min_s = UINT32_MAX;
#if ENABLE_BME280
    if (BME280_CHECK_INTERVAL_S < min_s) min_s = BME280_CHECK_INTERVAL_S;
#endif
#if ENABLE_BH1750
    if (BH1750_CHECK_INTERVAL_S < min_s) min_s = BH1750_CHECK_INTERVAL_S;
#endif
#if ENABLE_SCD41
    if (SCD41_CHECK_INTERVAL_S < min_s) min_s = SCD41_CHECK_INTERVAL_S;
#endif
    if (min_s == UINT32_MAX) min_s = 60;
    return min_s;
}

// Reporting thresholds from config
#if ENABLE_BME280
static int16_t  report_temp_threshold  = REPORT_TEMP_THRESHOLD;
static uint16_t report_hum_threshold   = REPORT_HUM_THRESHOLD;
static int16_t  report_press_threshold = REPORT_PRESS_THRESHOLD;
#endif
#if ENABLE_BH1750
static uint16_t report_illum_threshold = REPORT_ILLUM_THRESHOLD;
#endif
#if ENABLE_SCD41
static float    report_co2_threshold   = REPORT_CO2_THRESHOLD;
#endif
static uint8_t  report_batt_threshold  = REPORT_BATT_THRESHOLD;

// Global handles
static i2c_bus_handle_t i2c_bus = NULL;

// Custom functions
static void i2c_bus_init(void);
static void nvs_init(void);

static void one_time_sensor_task(void *pvParameters);
static void sensor_task_trigger_cb(uint8_t param);

static void zb_configure_reporting(void);
static void bind_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx);
static void zb_configure_binding(void);
static void esp_zb_task(void *pvParameter);
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct);
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message);

void app_main(void)
{
    ESP_LOGI(TAG, "System starting...");

    // Configure power management
    esp_pm_config_t pm_config = {
        .max_freq_mhz = PM_MAX_FREQ_MHZ,
        .min_freq_mhz = PM_MIN_FREQ_MHZ,
        .light_sleep_enable = PM_LIGHT_SLEEP_ENABLE,
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));

    // Initialize hardware
    nvs_init();
    i2c_bus_init();
#if ENABLE_BME280
    sensor_bme280_init(i2c_bus);
#endif
#if ENABLE_BH1750
    sensor_bh1750_init(i2c_bus);
#endif
#if ENABLE_SCD41
    sensor_scd41_init(i2c_bus);
#endif
    battery_init();

    // Create Zigbee stack task
    xTaskCreate(esp_zb_task, "Zigbee_task", 4096, NULL, 5, NULL);

    // Set up scheduled sensor task
    esp_zb_scheduler_alarm((esp_zb_callback_t)sensor_task_trigger_cb, 0, 2000);
}

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

static void nvs_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition is full or corrupted, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized successfully.");
}

static void zb_configure_reporting(void) {
    esp_zb_zcl_config_report_cmd_t report_cmd = {0};
    report_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    report_cmd.zcl_basic_cmd.dst_addr_u.addr_short = esp_zb_get_short_address();
    report_cmd.zcl_basic_cmd.src_endpoint = ZB_ENDPOINT;
    report_cmd.zcl_basic_cmd.dst_endpoint = ZB_ENDPOINT;

    esp_zb_lock_acquire(portMAX_DELAY);

#if ENABLE_BME280
    // Temperature Reporting
    report_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
    esp_zb_zcl_config_report_record_t temp_record = {
        .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
        .attributeID = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        .attrType = ESP_ZB_ZCL_ATTR_TYPE_S16,
        .min_interval = ZB_REPORT_BME280_MIN_INTERVAL_S,
        .max_interval = ZB_REPORT_BME280_MAX_INTERVAL_S,
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
        .min_interval = ZB_REPORT_BME280_MIN_INTERVAL_S,
        .max_interval = ZB_REPORT_BME280_MAX_INTERVAL_S,
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
        .min_interval = ZB_REPORT_BME280_MIN_INTERVAL_S,
        .max_interval = ZB_REPORT_BME280_MAX_INTERVAL_S,
        .reportable_change = &report_press_threshold,
    };
    report_cmd.record_field = &press_record;
    esp_zb_zcl_config_report_cmd_req(&report_cmd);
#endif

#if ENABLE_BH1750
    // Illuminance Reporting
    report_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT;
    esp_zb_zcl_config_report_record_t illum_record = {
        .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
        .attributeID = ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID,
        .attrType = ESP_ZB_ZCL_ATTR_TYPE_U16,
        .min_interval = ZB_REPORT_BH1750_MIN_INTERVAL_S,
        .max_interval = ZB_REPORT_BH1750_MAX_INTERVAL_S,
        .reportable_change = &report_illum_threshold,
    };
    report_cmd.record_field = &illum_record;
    esp_zb_zcl_config_report_cmd_req(&report_cmd);
#endif

#if ENABLE_SCD41
    // Carbon Dioxide Reporting
    report_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT;
    esp_zb_zcl_config_report_record_t co2_record = {
        .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
        .attributeID = ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID,
        .attrType = ESP_ZB_ZCL_ATTR_TYPE_SINGLE,
        .min_interval = ZB_REPORT_SCD41_MIN_INTERVAL_S,
        .max_interval = ZB_REPORT_SCD41_MAX_INTERVAL_S,
        .reportable_change = &report_co2_threshold,
    };
    report_cmd.record_field = &co2_record;
    esp_zb_zcl_config_report_cmd_req(&report_cmd);
#endif

    // Battery Reporting
    report_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG;
    esp_zb_zcl_config_report_record_t batt_record = {
        .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
        .attributeID = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
        .attrType = ESP_ZB_ZCL_ATTR_TYPE_U8,
        .min_interval = ZB_REPORT_BATT_MIN_INTERVAL_S,
        .max_interval = ZB_REPORT_BATT_MAX_INTERVAL_S,
        .reportable_change = &report_batt_threshold,
    };
    report_cmd.record_field = &batt_record;
    esp_zb_zcl_config_report_cmd_req(&report_cmd);

    esp_zb_lock_release();
    ESP_LOGI(TAG, "Automatic reporting configured.");
}

static void bind_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx) {
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Bind success!");
    } else {
        ESP_LOGW(TAG, "Bind failed with status: 0x%02x", zdo_status);
    }
}

static void zb_configure_binding(void) {
    esp_zb_zdo_bind_req_param_t bind_req = {0};

    esp_zb_get_long_address(bind_req.src_address);
    bind_req.src_endp = ZB_ENDPOINT;
    bind_req.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
    esp_zb_ieee_address_by_short(0x0000, bind_req.dst_address_u.addr_long);

    uint16_t clusters_to_bind[] = {
        ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
#if ENABLE_BME280
        ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,
#endif
#if ENABLE_BH1750
        ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT,
#endif
#if ENABLE_SCD41
        ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT,
#endif
    };

    for (int i = 0; i < sizeof(clusters_to_bind)/sizeof(uint16_t); i++) {
        bind_req.cluster_id = clusters_to_bind[i];
        esp_zb_zdo_device_bind_req(&bind_req, bind_cb, NULL);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "Binding configured.");
}

static void esp_zb_task(void *pvParameter) {
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,
        .install_code_policy = false,
        .nwk_cfg.zed_cfg = {
            .ed_timeout = ZB_ED_AGING_TIMEOUT,
            .keep_alive = ZB_ED_KEEP_ALIVE_MS,
        },
    };
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_set_rx_on_when_idle(false);
    esp_zb_sleep_enable(true);
    esp_zb_sleep_set_threshold(ZB_SLEEP_THRESHOLD_MS);

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

    // Create power cluster
    esp_zb_attribute_list_t *power_cfg_cluster = esp_zb_power_config_cluster_create(NULL);
    uint8_t battery_percentage = 200;
    esp_zb_power_config_cluster_add_attr(power_cfg_cluster,
                            ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
                            &battery_percentage);
    esp_zb_cluster_list_add_power_config_cluster(cluster_list, power_cfg_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

#if ENABLE_BME280
    // Create temperature sensor cluster
    esp_zb_temperature_meas_cluster_cfg_t temp_cfg = {
        .min_value = -1000,
        .max_value = 5000,
    };
    esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list,
        esp_zb_temperature_meas_cluster_create(&temp_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Create humidity sensor cluster
    esp_zb_humidity_meas_cluster_cfg_t hum_cfg = {
        .min_value = 0,
        .max_value = 10000
    };
    esp_zb_cluster_list_add_humidity_meas_cluster(cluster_list,
        esp_zb_humidity_meas_cluster_create(&hum_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Create pressure sensor cluster
    esp_zb_pressure_meas_cluster_cfg_t press_cfg = {
        .min_value = 300,
        .max_value = 1100,
    };
    esp_zb_cluster_list_add_pressure_meas_cluster(cluster_list,
        esp_zb_pressure_meas_cluster_create(&press_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
#endif

#if ENABLE_BH1750
    // Create illuminance sensor cluster
    esp_zb_illuminance_meas_cluster_cfg_t illum_cfg = {
        .measured_value = 0,
        .min_value = 0,
        .max_value = 0xFFFE,
    };
    esp_zb_cluster_list_add_illuminance_meas_cluster(cluster_list,
        esp_zb_illuminance_meas_cluster_create(&illum_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
#endif

#if ENABLE_SCD41
    // Create CO2 sensor cluster
    esp_zb_carbon_dioxide_measurement_cluster_cfg_t co2_cfg = {
        .measured_value = 0.000400f,     // 400 ppm
        .min_measured_value = 0.000400f, // 400 ppm
        .max_measured_value = 0.040000f, // 40000 ppm
    };
    esp_zb_cluster_list_add_carbon_dioxide_measurement_cluster(cluster_list,
        esp_zb_carbon_dioxide_measurement_cluster_create(&co2_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
#endif

    // Create endpoint
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t ep_config = {
        .endpoint = ZB_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, cluster_list, ep_config);

    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_device_register(ep_list);
    esp_zb_set_primary_network_channel_set(1l << ZB_PRIMARY_CHANNEL);
    ESP_ERROR_CHECK(esp_zb_start(false));

    esp_zb_stack_main_loop();
}

static void one_time_sensor_task(void *pvParameters) {
    int64_t now_s = esp_timer_get_time() / 1000000ULL;

#if ENABLE_BME280
    static int64_t last_bme280_s = -BME280_CHECK_INTERVAL_S;
    bool do_bme280 = (now_s - last_bme280_s >= BME280_CHECK_INTERVAL_S);
    float temp = 0.0f, hum = 0.0f, press = 0.0f;
    esp_err_t bme_err = ESP_FAIL;
    if (do_bme280) {
        last_bme280_s = now_s;
        bme_err = sensor_bme280_trigger();
    }
#endif

#if ENABLE_BH1750
    static int64_t last_bh1750_s = -BH1750_CHECK_INTERVAL_S;
    bool do_bh1750 = (now_s - last_bh1750_s >= BH1750_CHECK_INTERVAL_S);
    float lux = 0.0f;
    esp_err_t bh_err = ESP_FAIL;
    if (do_bh1750) {
        last_bh1750_s = now_s;
        bh_err = sensor_bh1750_trigger();
    }
#endif

#if ENABLE_SCD41
    static int64_t last_scd41_s = -SCD41_CHECK_INTERVAL_S;
    bool do_scd41 = (now_s - last_scd41_s >= SCD41_CHECK_INTERVAL_S);
    uint16_t co2_ppm = 0;
    esp_err_t scd_err = ESP_FAIL;
    if (do_scd41) {
        last_scd41_s = now_s;
        sensor_scd41_wake_up();
        scd_err = sensor_scd41_trigger();
    }
#endif

    // Wait 180 ms if any sensor was triggered
    bool waited = false;
#if ENABLE_BME280
    if (do_bme280) waited = true;
#endif
#if ENABLE_BH1750
    if (do_bh1750) waited = true;
#endif
    if (waited) {
        vTaskDelay(pdMS_TO_TICKS(180));
    }

#if ENABLE_BME280
    if (do_bme280) {
        if (bme_err == ESP_OK && sensor_bme280_read(&temp, &hum, &press) == ESP_OK) {
            int16_t zcl_temp = (int16_t)(temp * 100);
            esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                                         ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
                                         &zcl_temp, false);

            uint16_t zcl_hum = (uint16_t)(hum * 100);
            esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
                                         ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
                                         &zcl_hum, false);

            int16_t zcl_press = (int16_t)press;
            esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,
                                         ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID,
                                         &zcl_press, false);
        } else {
            ESP_LOGW(TAG, "BME280 measurement skipped or failed");
        }
    }
#endif

#if ENABLE_BH1750
    if (do_bh1750) {
        if (bh_err == ESP_OK && sensor_bh1750_read(&lux) == ESP_OK) {
            uint16_t zcl_lux = 0;
            if (lux >= 1.0f) {
                float measured = 10000.0f * log10f(lux) + 1.0f;
                if (measured > 0xFFFE) measured = 0xFFFE;
                zcl_lux = (uint16_t)measured;
            } else {
                zcl_lux = 0;
            }
            esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT,
                                         ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID,
                                         &zcl_lux, false);
        } else {
            ESP_LOGW(TAG, "BH1750 measurement skipped or failed");
        }
    }
#endif

#if ENABLE_SCD41
    if (do_scd41) {
        // SCD41 single-shot takes ~5000 ms. If we waited 180 ms for BME280/BH1750, wait remaining 4820 ms.
        vTaskDelay(pdMS_TO_TICKS(waited ? 4820 : 5000));
        if (scd_err == ESP_OK && sensor_scd41_read(&co2_ppm) == ESP_OK) {
            float zcl_co2 = (float)co2_ppm / 1000000.0f;
            esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT,
                                         ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID,
                                         &zcl_co2, false);
            ESP_LOGI(TAG, "Readings: CO2: %u ppm", co2_ppm);
        } else {
            ESP_LOGW(TAG, "SCD41 measurement skipped or failed");
        }
        sensor_scd41_power_down();
    }
#endif

    int battery_voltage = 0;
    uint8_t battery_percentage = 0;
    battery_read(&battery_voltage, &battery_percentage);

#if ENABLE_BME280 && ENABLE_BH1750
    if (do_bme280 && do_bh1750) {
        ESP_LOGI(TAG, "Readings: Temp: %.2f C, Humidity: %.2f %%, Pressure: %.2f kPa, Light: %.1f lux, Bat: %d %%",
                 temp, hum, press/10.0f, lux, battery_percentage);
    } else if (do_bme280) {
        ESP_LOGI(TAG, "Readings: Temp: %.2f C, Humidity: %.2f %%, Pressure: %.2f kPa, Bat: %d %%",
                 temp, hum, press/10.0f, battery_percentage);
    } else if (do_bh1750) {
        ESP_LOGI(TAG, "Readings: Light: %.1f lux, Bat: %d %%", lux, battery_percentage);
    } else {
        ESP_LOGI(TAG, "Readings: Bat: %d %% (no sensor due)", battery_percentage);
    }
#elif ENABLE_BME280
    if (do_bme280) {
        ESP_LOGI(TAG, "Readings: Temp: %.2f C, Humidity: %.2f %%, Pressure: %.2f kPa, Bat: %d %%",
                 temp, hum, press/10.0f, battery_percentage);
    } else {
        ESP_LOGI(TAG, "Readings: Bat: %d %% (no sensor due)", battery_percentage);
    }
#elif ENABLE_BH1750
    if (do_bh1750) {
        ESP_LOGI(TAG, "Readings: Light: %.1f lux, Bat: %d %%", lux, battery_percentage);
    } else {
        ESP_LOGI(TAG, "Readings: Bat: %d %% (no sensor due)", battery_percentage);
    }
#else
    ESP_LOGI(TAG, "Readings: Bat: %d %%", battery_percentage);
#endif

    uint8_t zcl_batt = battery_percentage * 2;
    esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
                                 &zcl_batt, false);

    esp_zb_stack_main_loop_iteration();

    ESP_LOGI(TAG, "Sensor task finished, deleting...");
    vTaskDelete(NULL);
}

static void sensor_task_trigger_cb(uint8_t param) {
    xTaskCreate(one_time_sensor_task, "sensor_task", 4096, NULL, 3, NULL);
    esp_zb_scheduler_alarm((esp_zb_callback_t)sensor_task_trigger_cb, 0, get_base_interval_s() * 1000);
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
    esp_err_t err = esp_zb_bdb_start_top_level_commissioning(mode_mask);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start Zigbee commissioning via alarm, error: 0x%x", err);
    }
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            ESP_LOGI(TAG, "Initialize Zigbee stack.");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
            break;
        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (err_status == ESP_OK) {
                if (esp_zb_bdb_is_factory_new()) {
                    ESP_LOGI(TAG, "Device is factory new, start network steering...");
                    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                } else {
                    ESP_LOGI(TAG, "Device rebooted, already in network.");
                    zb_configure_reporting();
                    zb_configure_binding();
                }
            }
            break;
        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err_status == ESP_OK) {
                ESP_LOGI(TAG, "Joined network successfully.");
                zb_configure_reporting();
                zb_configure_binding();
            } else {
                ESP_LOGW(TAG, "Network steering failed, retrying in 1s.");
                esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            }
            break;
        case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
            ESP_LOGI(TAG, "Zigbee stack indicates it can sleep now.");
            esp_zb_sleep_now();
            break;
        default:
            break;
    }
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message) {
    esp_err_t ret = ESP_OK;

    switch (callback_id) {
        case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
            ESP_LOGI(TAG, "Receive ZCL set attribute request.");
            break;
        case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
            ESP_LOGI(TAG, "Receive ZCL command default response.");
            break;
        default:
            ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback.", callback_id);
            break;
    }
    return ret;
}
