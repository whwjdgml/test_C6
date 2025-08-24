#ifndef PROPORTIONAL_BATTERY_HEATER_H
#define PROPORTIONAL_BATTERY_HEATER_H

#include "battery_heater.h"

// 비례 제어 배터리 히터 클래스
class ProportionalBatteryHeater : public BatteryHeater {
private:
    static const char* TAG;
    
    // 비례 제어 파라미터
    float target_temperature = 5.0f;        // 목표 온도 (°C)
    float proportional_gain = 15.0f;        // 비례 게인 Kp
    bool proportional_mode_enabled = true;  // 비례 제어 활성화
    
    // 튜닝 가능 파라미터
    float min_duty = 0.0f;                  // 최소 듀티 (%)
    float max_duty = 100.0f;                // 최대 듀티 (%)
    float dead_zone = 0.5f;                 // 데드존 (±0.5°C)
    
    // 성능 모니터링
    float avg_error = 0.0f;
    float max_error = 0.0f;
    uint32_t control_updates = 0;
    
    // 내부 메서드
    uint8_t calculateProportionalDuty(float current_temp);
    void updateStatistics(float error);

public:
    ProportionalBatteryHeater(NTCSensor* sensor, gpio_num_t stepup_pin, gpio_num_t pwm_output_pin);
    
    // 비례 제어 업데이트
    bool updateProportionalHeater();
    
    // 파라미터 설정
    void setTargetTemperature(float target) { target_temperature = target; }
    void setProportionalGain(float kp) { proportional_gain = kp; }
    void setDutyLimits(float min_pct, float max_pct) { 
        min_duty = min_pct; 
        max_duty = max_pct; 
    }
    void setDeadZone(float zone) { dead_zone = zone; }
    void enableProportionalMode(bool enable) { proportional_mode_enabled = enable; }
    
    // 상태 조회
    float getTargetTemperature() const { return target_temperature; }
    float getProportionalGain() const { return proportional_gain; }
    float getAverageError() const { return avg_error; }
    float getMaxError() const { return max_error; }
    
    // 성능 분석
    void printControlParameters();
    void printPerformanceStatistics();
    void resetStatistics();
    
    // 실시간 튜닝 (시리얼 명령어 지원)
    void handleTuningCommand(const char* command);
};

#endif // PROPORTIONAL_BATTERY_HEATER_H