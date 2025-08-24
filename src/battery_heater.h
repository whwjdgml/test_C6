#ifndef BATTERY_HEATER_H
#define BATTERY_HEATER_H

#include "ntc_sensor.h"
#include "sensor_config.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class BatteryHeater {
protected:
    static const char* TAG;
    
    // 하드웨어 설정
    NTCSensor* ntc_sensor;
    gpio_num_t stepup_en_pin;
    gpio_num_t pwm_pin;
    
    // LEDC PWM 설정
    ledc_channel_t ledc_channel;
    ledc_timer_t ledc_timer;
    static constexpr ledc_mode_t ledc_mode = LEDC_LOW_SPEED_MODE;
    
    // 히터 상태
    heater_state_t current_state;
    heater_state_t previous_state;
    bool stepup_enabled;
    bool initialized;
    
    // 온도 히스테리시스 제어
    float last_battery_temp;
    uint32_t last_update_ms;
    uint32_t state_change_count;
    
    // 통계
    uint32_t total_heating_time_ms;
    uint32_t heating_cycles;
    
    // 내부 메서드
    bool initPWM();
    bool enableStepUpConverter();
    void disableStepUpConverter();
    void setPWMDuty(uint8_t duty_percent);
    heater_state_t calculateNewState(float battery_temp);
    void updateHeaterOutput(heater_state_t new_state);
    uint8_t getStatePWMDuty(heater_state_t state);
    const char* getStateName(heater_state_t state);
    float getStatePowerConsumption(heater_state_t state) const;

public:
    BatteryHeater(NTCSensor* sensor, gpio_num_t stepup_pin, gpio_num_t pwm_output_pin,
                  ledc_channel_t channel = LEDC_CHANNEL_0, ledc_timer_t timer = LEDC_TIMER_0);
    ~BatteryHeater();
    
    bool init();
    bool updateHeater();
    void emergencyShutdown();
    
    // 상태 조회
    heater_state_t getCurrentState() const { return current_state; }
    float getLastTemperature() const { return last_battery_temp; }
    bool isStepUpEnabled() const { return stepup_enabled; }
    float getCurrentPowerConsumption() const;
    
    // 통계
    uint32_t getTotalHeatingTimeMs() const { return total_heating_time_ms; }
    uint32_t getHeatingCycles() const { return heating_cycles; }
    void resetStatistics();
    
    // 디버깅
    void printStatus();
    void printStatistics();
    
    void deinit();
};

#endif // BATTERY_HEATER_H