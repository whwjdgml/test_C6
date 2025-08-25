#include "ntc_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

const char* NTCSensor::TAG = "NTC_SENSOR";

NTCSensor::NTCSensor(adc_channel_t channel, gpio_num_t power_control_pin) 
    : adc1_handle(nullptr), adc1_cali_handle(nullptr), 
      adc_channel(channel), power_pin(power_control_pin),
      adc_calibrated(false), initialized(false) {
}

NTCSensor::~NTCSensor() {
    deinit();
}

bool NTCSensor::init() {
    ESP_LOGI(TAG, "NTC 온도센서 초기화 시작 (100kΩ 3950 1%)");
    
    // 전원 제어 핀 초기화 (LOW = 전원 OFF)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << power_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "전원 제어 핀 초기화 실패: %s", esp_err_to_name(ret));
        return false;
    }
    
    // 초기에는 전원 OFF
    gpio_set_level(power_pin, 0);
    
    // ADC 초기화
    if (!initADC()) {
        ESP_LOGE(TAG, "ADC 초기화 실패");
        return false;
    }
    
    // 테스트 측정
    float test_temp;
    if (readTemperature(&test_temp)) {
        ESP_LOGI(TAG, "✅ NTC 센서 초기화 완료 - 현재 온도: %.2f°C", test_temp);
        initialized = true;
        return true;
    } else {
        ESP_LOGE(TAG, "❌ NTC 센서 테스트 측정 실패");
        return false;
    }
}

bool NTCSensor::initADC() {
    // ADC1 초기화
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    
    esp_err_t ret = adc_oneshot_new_unit(&init_config1, &adc1_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC1 유닛 생성 실패: %s", esp_err_to_name(ret));
        return false;
    }
    
    // ADC 채널 구성
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,      // 3.3V까지 측정 가능
        .bitwidth = ADC_BITWIDTH_12,   // 12비트 해상도 (0-4095)
    };
    
    ret = adc_oneshot_config_channel(adc1_handle, adc_channel, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC 채널 구성 실패: %s", esp_err_to_name(ret));
        return false;
    }
    
    // ADC 캘리브레이션
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = adc_channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle);
    if (ret == ESP_OK) {
        adc_calibrated = true;
        ESP_LOGI(TAG, "ADC 캘리브레이션 적용됨");
    } else {
        ESP_LOGW(TAG, "ADC 캘리브레이션 실패, raw 값 사용: %s", esp_err_to_name(ret));
        adc_calibrated = false;
    }
    
    return true;
}

void NTCSensor::enablePower() {
    gpio_set_level(power_pin, 1);  // 전원 ON
    vTaskDelay(pdMS_TO_TICKS(POWER_ON_DELAY_MS));  // 안정화 대기
}

void NTCSensor::disablePower() {
    gpio_set_level(power_pin, 0);  // 전원 OFF
}

bool NTCSensor::readTemperature(float* temperature) {
    if (!initialized) {
        ESP_LOGE(TAG, "센서가 초기화되지 않음");
        return false;
    }
    
    enablePower();  // 측정 시에만 전원 ON
    
    int raw_voltage = readRawVoltage();
    if (raw_voltage < 0) {
        disablePower();
        return false;
    }
    
    *temperature = calculateTemperature(raw_voltage);
    
    disablePower();  // 측정 후 전원 OFF
    
    // 유효성 검증 (-40°C ~ 85°C)
    if (*temperature < -40.0f || *temperature > 85.0f) {
        ESP_LOGW(TAG, "온도 값이 범위를 벗어남: %.2f°C", *temperature);
        return false;
    }
    
    ESP_LOGD(TAG, "NTC 온도: %.2f°C (raw: %d)", *temperature, raw_voltage);
    return true;
}

int NTCSensor::readRawVoltage() {
    const int num_samples = 10;
    int voltage_sum = 0;
    int valid_samples = 0;
    
    // 다중 샘플링으로 노이즈 제거
    for (int i = 0; i < num_samples; i++) {
        int adc_reading;
        esp_err_t ret = adc_oneshot_read(adc1_handle, adc_channel, &adc_reading);
        
        if (ret == ESP_OK) {
            int voltage;
            if (adc_calibrated) {
                ret = adc_cali_raw_to_voltage(adc1_cali_handle, adc_reading, &voltage);
                if (ret == ESP_OK) {
                    voltage_sum += voltage;
                    valid_samples++;
                }
            } else {
                // 캘리브레이션 없이 계산: (raw / 4095) * 3300mV
                voltage = (adc_reading * 3300) / 4095;
                voltage_sum += voltage;
                valid_samples++;
            }
        }
        
        if (i < num_samples - 1) {
            vTaskDelay(pdMS_TO_TICKS(1));  // 1ms 간격
        }
    }
    
    if (valid_samples == 0) {
        ESP_LOGE(TAG, "ADC 읽기 실패");
        return -1;
    }
    
    return voltage_sum / valid_samples;
}

float NTCSensor::calculateTemperature(int raw_voltage) {
    // 전압을 저항으로 변환 (전압분배 공식)
    float voltage = raw_voltage / 1000.0f;  // mV를 V로 변환
    
    if (voltage >= VCC_VOLTAGE) {
        ESP_LOGW(TAG, "전압 포화: %.3fV", voltage);
        return -40.0f;  // 최소값 반환
    }
    
    // NTC 저항 계산: R_ntc = R_pullup * V_ntc / (V_cc - V_ntc)
    float ntc_resistance = PULLUP_RESISTANCE * voltage / (VCC_VOLTAGE - voltage);
    
    // Steinhart-Hart 간소화 공식을 사용한 온도 계산
    // 1/T = 1/T0 + (1/B) * ln(R/R0)
    float temp_kelvin = 1.0f / (
        (1.0f / (NTC_TEMPERATURE_NOMINAL + 273.15f)) + 
        (1.0f / NTC_B_COEFFICIENT) * log(ntc_resistance / NTC_RESISTANCE_NOMINAL)
    );
    
    float temp_celsius = temp_kelvin - 273.15f;
    
    ESP_LOGV(TAG, "전압: %.3fV, 저항: %.0fΩ, 온도: %.2f°C", 
             voltage, ntc_resistance, temp_celsius);
    
    return temp_celsius;
}

int NTCSensor::getRawVoltage() {
    if (!initialized) return -1;
    
    enablePower();
    int raw = readRawVoltage();
    disablePower();
    
    return raw;
}

void NTCSensor::printCalibrationInfo() {
    ESP_LOGI(TAG, "=== NTC 센서 캘리브레이션 정보 ===");
    ESP_LOGI(TAG, "기준 저항: %.0fΩ @ %.1f°C", NTC_RESISTANCE_NOMINAL, NTC_TEMPERATURE_NOMINAL);
    ESP_LOGI(TAG, "B 계수: %.0f", NTC_B_COEFFICIENT);
    ESP_LOGI(TAG, "풀업 저항: %.0fΩ", PULLUP_RESISTANCE);
    ESP_LOGI(TAG, "기준 전압: %.1fV", VCC_VOLTAGE);
    ESP_LOGI(TAG, "ADC 캘리브레이션: %s", adc_calibrated ? "사용됨" : "미사용");
}

void NTCSensor::deinit() {
    if (adc1_cali_handle) {
        adc_cali_delete_scheme_curve_fitting(adc1_cali_handle);
        adc1_cali_handle = nullptr;
    }
    
    if (adc1_handle) {
        adc_oneshot_del_unit(adc1_handle);
        adc1_handle = nullptr;
    }
    
    initialized = false;
    ESP_LOGI(TAG, "NTC 센서 정리 완료");
}