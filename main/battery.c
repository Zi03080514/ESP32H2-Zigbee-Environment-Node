#include "battery.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

#if __has_include("config.h")
#include "config.h"
#else
#include "config.h.example"
#endif

static const char *TAG = "BATTERY";

static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_cali_handle_t adc1_cali_handle = NULL;

static uint8_t calc_battery_percentage(uint32_t voltage_mv) {
    static const struct {
        uint16_t mv;
        uint8_t pct;
    } curve[] = {
        {3200, 0},   {3400, 10},  {3600, 20},
        {3700, 40},  {3800, 60},  {3900, 80},
        {4000, 90},  {4200, 100}
    };

    const int num_points = sizeof(curve) / sizeof(curve[0]);

    if (voltage_mv <= curve[0].mv) {
        return 0;
    }
    if (voltage_mv >= curve[num_points - 1].mv) {
        return 100;
    }

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

esp_err_t battery_init(void) {
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_config1, &adc1_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
        return ret;
    }

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ret = adc_oneshot_config_channel(adc1_handle, BATTERY_ADC_CHANNEL, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel: %s", esp_err_to_name(ret));
        return ret;
    }

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = BATTERY_ADC_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ADC calibration initialization failed: %s", esp_err_to_name(ret));
        // Continue even if calibration fails, fallback can be used
    }

    ESP_LOGI(TAG, "Battery monitoring initialized.");
    return ESP_OK;
}

esp_err_t battery_read(int *voltage_mv, uint8_t *percentage) {
    if (!adc1_handle) {
        return ESP_ERR_INVALID_STATE;
    }

    int raw = 0;
    esp_err_t ret = adc_oneshot_read(adc1_handle, BATTERY_ADC_CHANNEL, &raw);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ADC read failed: %s", esp_err_to_name(ret));
        return ret;
    }

    int mv = 0;
    if (adc1_cali_handle) {
        ret = adc_cali_raw_to_voltage(adc1_cali_handle, raw, &mv);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "ADC raw to voltage failed: %s", esp_err_to_name(ret));
            return ret;
        }
    } else {
        mv = (raw * 3300) / 4095;
    }

    mv *= BATTERY_ADC_DIVISION_FACTOR;

    if (voltage_mv) {
        *voltage_mv = mv;
    }
    if (percentage) {
        *percentage = calc_battery_percentage((uint32_t)mv);
    }

    return ESP_OK;
}

