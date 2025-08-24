#ifndef NTC_SENSOR_H
#define NTC_SENSOR_H

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <cmath>

class NTCSensor {
private:
    static const char* TAG;
    
    // ADC 설정
    adc_oneshot_unit_handle_t adc1_handle;
    adc_cali_handle_t adc1_cali_handle;
    adc_channel_t adc_channel;
    gpio_num_t power_pin;
    
    // NTC 100k 3950 1% 센서 파라미터
    static constexpr float NTC_RESISTANCE_NOMINAL = 100000.0f;  // 25°C에서 100kΩ
    static constexpr float NTC_TEMPERATURE_NOMINAL = 25.0f;     // 기준 온도 25°C
    static constexpr float NTC_B_COEFFICIENT = 3950.0f;        // B 계수
    static constexpr float PULLUP_RESISTANCE = 10000.0f;       // 10kΩ 풀업 저항
    static constexpr float VCC_VOLTAGE = 3.3f;                 // 기준 전압 3.3V
    
    // 전력 절약 설정
    static constexpr uint32_t POWER_ON_DELAY_MS = 10;          // 전원 켜고 안정화 시간
    
    bool adc_calibrated;
    bool initialized;
    
    // 내부 메서드
    int readRawVoltage();
    float calculateTemperature(int raw_voltage);
    bool initADC();
    void enablePower();
    void disablePower();

public:
    NTCSensor(adc_channel_t channel, gpio_num_t power_control_pin);
    ~NTCSensor();
    
    bool init();
    bool readTemperature(float* temperature);
    bool isInitialized() const { return initialized; }
    void deinit();
    
    // 디버깅용
    void printCalibrationInfo();
    int getRawVoltage();
};

#endif // NTC_SENSOR_H