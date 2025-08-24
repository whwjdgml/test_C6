// 동적 배터리 히터 제어 구현 예시들
#include "battery_heater.h"

// 1. 비례 제어 (P Control) - 가장 간단한 동적 제어
class ProportionalHeaterController {
private:
    static constexpr float TARGET_TEMP = 5.0f;      // 목표 온도 5°C
    static constexpr float Kp = 15.0f;              // 비례 게인
    static constexpr float MIN_DUTY = 0.0f;         // 최소 듀티 0%
    static constexpr float MAX_DUTY = 100.0f;       // 최대 듀티 100%
    
public:
    uint8_t calculateDuty(float current_temp) {
        // 온도 오차 계산
        float error = TARGET_TEMP - current_temp;
        
        // 비례 제어: PWM = Kp × error
        float duty = Kp * error;
        
        // 듀티 제한
        if (duty < MIN_DUTY) duty = MIN_DUTY;
        if (duty > MAX_DUTY) duty = MAX_DUTY;
        
        ESP_LOGD("P_CTRL", "온도: %.2f°C, 오차: %.2f°C, 듀티: %.1f%%", 
                current_temp, error, duty);
        
        return (uint8_t)duty;
    }
};

// 2. PID 제어 - 가장 정교한 제어
class PIDHeaterController {
private:
    static constexpr float TARGET_TEMP = 5.0f;
    static constexpr float Kp = 20.0f;              // 비례 게인
    static constexpr float Ki = 0.5f;               // 적분 게인  
    static constexpr float Kd = 2.0f;               // 미분 게인
    
    float integral_error = 0.0f;
    float previous_error = 0.0f;
    uint32_t last_time_ms = 0;
    
public:
    uint8_t calculateDuty(float current_temp) {
        uint32_t current_time = esp_timer_get_time() / 1000;
        float dt = (current_time - last_time_ms) / 1000.0f; // 초 단위
        
        if (dt <= 0) return 0; // 시간 차이가 없으면 0 반환
        
        // 현재 오차
        float error = TARGET_TEMP - current_temp;
        
        // 적분항 (누적 오차)
        integral_error += error * dt;
        
        // 적분 와인드업 방지
        if (integral_error > 100.0f) integral_error = 100.0f;
        if (integral_error < -100.0f) integral_error = -100.0f;
        
        // 미분항 (오차 변화율)
        float derivative = (error - previous_error) / dt;
        
        // PID 출력 계산
        float output = Kp * error + Ki * integral_error + Kd * derivative;
        
        // 듀티 제한
        if (output < 0) output = 0;
        if (output > 100) output = 100;
        
        // 다음 계산을 위해 저장
        previous_error = error;
        last_time_ms = current_time;
        
        ESP_LOGV("PID_CTRL", "T:%.2f E:%.2f I:%.2f D:%.2f → %.1f%%", 
                current_temp, error, integral_error, derivative, output);
        
        return (uint8_t)output;
    }
    
    void reset() {
        integral_error = 0.0f;
        previous_error = 0.0f;
        last_time_ms = 0;
    }
};

// 3. 적응형 제어 - 환경에 따라 자동 적응
class AdaptiveHeaterController {
private:
    static constexpr float BASE_TARGET = 5.0f;      // 기본 목표 온도
    static constexpr float Kp_BASE = 15.0f;         // 기본 게인
    
    float adaptive_target = BASE_TARGET;
    float adaptive_gain = Kp_BASE;
    float ambient_temp = 20.0f;                     // 주변 온도 추정
    
    // 성능 지표
    float avg_power_consumption = 0.0f;
    uint32_t control_cycles = 0;
    
public:
    uint8_t calculateDuty(float battery_temp, float external_temp = NAN) {
        // 외부 온도가 있으면 주변 온도 업데이트
        if (!isnan(external_temp)) {
            ambient_temp = external_temp;
        }
        
        // 주변 온도에 따른 목표 온도 적응
        if (ambient_temp < -20.0f) {
            adaptive_target = 8.0f;     // 극한추위: 높은 목표온도
            adaptive_gain = 25.0f;      // 강한 제어
        } else if (ambient_temp < -10.0f) {
            adaptive_target = 6.0f;     // 추위: 중간 목표온도  
            adaptive_gain = 20.0f;
        } else if (ambient_temp < 0.0f) {
            adaptive_target = 5.0f;     // 기본
            adaptive_gain = 15.0f;
        } else {
            adaptive_target = 3.0f;     // 따뜻함: 낮은 목표온도
            adaptive_gain = 10.0f;      // 약한 제어
        }
        
        // 기본 비례 제어
        float error = adaptive_target - battery_temp;
        float duty = adaptive_gain * error;
        
        // 듀티 제한
        if (duty < 0) duty = 0;
        if (duty > 100) duty = 100;
        
        // 성능 통계 업데이트
        control_cycles++;
        float power = (duty * duty / 10000.0f) * 2.5f; // PWM 제곱 법칙
        avg_power_consumption = (avg_power_consumption * (control_cycles-1) + power) / control_cycles;
        
        ESP_LOGD("ADAPTIVE", "주변:%.1f°C 목표:%.1f°C 배터리:%.1f°C → %.1f%% (평균전력:%.2fW)",
                ambient_temp, adaptive_target, battery_temp, duty, avg_power_consumption);
        
        return (uint8_t)duty;
    }
    
    float getAveragePowerConsumption() const { return avg_power_consumption; }
    void resetStatistics() { 
        avg_power_consumption = 0.0f; 
        control_cycles = 0; 
    }
};

// 4. 스마트 제어 - 예측 기반 제어
class PredictiveHeaterController {
private:
    static constexpr int TEMP_HISTORY_SIZE = 10;
    float temp_history[TEMP_HISTORY_SIZE] = {0};
    int history_index = 0;
    bool history_full = false;
    
    float target_temp = 5.0f;
    float predicted_temp = 0.0f;
    
public:
    uint8_t calculateDuty(float current_temp) {
        // 온도 이력 저장
        temp_history[history_index] = current_temp;
        history_index = (history_index + 1) % TEMP_HISTORY_SIZE;
        if (history_index == 0) history_full = true;
        
        // 온도 변화 경향 분석
        float temp_trend = calculateTemperatureTrend();
        
        // 다음 온도 예측 (1분 후)
        predicted_temp = current_temp + temp_trend * 60.0f; // 60초 후
        
        // 예측 온도 기반 제어
        float predicted_error = target_temp - predicted_temp;
        float current_error = target_temp - current_temp;
        
        // 예측과 현재 오차를 결합
        float combined_error = 0.7f * current_error + 0.3f * predicted_error;
        
        float duty = 15.0f * combined_error;
        
        // 듀티 제한
        if (duty < 0) duty = 0;
        if (duty > 100) duty = 100;
        
        ESP_LOGD("PREDICT", "현재:%.2f 예측:%.2f 경향:%.3f → %.1f%%", 
                current_temp, predicted_temp, temp_trend, duty);
        
        return (uint8_t)duty;
    }
    
private:
    float calculateTemperatureTrend() {
        if (!history_full && history_index < 3) return 0.0f;
        
        int samples = history_full ? TEMP_HISTORY_SIZE : history_index;
        if (samples < 2) return 0.0f;
        
        // 최소 제곱법으로 기울기 계산 (간소화 버전)
        float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
        
        for (int i = 0; i < samples; i++) {
            int idx = (history_index - samples + i + TEMP_HISTORY_SIZE) % TEMP_HISTORY_SIZE;
            float x = i;  // 시간 (상대적)
            float y = temp_history[idx];  // 온도
            
            sum_x += x;
            sum_y += y;
            sum_xy += x * y;
            sum_x2 += x * x;
        }
        
        // 기울기 = (n*sum_xy - sum_x*sum_y) / (n*sum_x2 - sum_x*sum_x)
        float n = samples;
        float slope = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);
        
        return slope; // °C/측정주기
    }
};

// 5. BatteryHeater 클래스에 동적 제어 추가하는 방법
class EnhancedBatteryHeater : public BatteryHeater {
private:
    enum ControlMode {
        CONTROL_FIXED,      // 기존 고정 듀티
        CONTROL_PROPORTIONAL, // 비례 제어
        CONTROL_PID,        // PID 제어  
        CONTROL_ADAPTIVE,   // 적응형 제어
        CONTROL_PREDICTIVE  // 예측 제어
    };
    
    ControlMode current_mode = CONTROL_FIXED;
    
    ProportionalHeaterController p_controller;
    PIDHeaterController pid_controller;
    AdaptiveHeaterController adaptive_controller;
    PredictiveHeaterController predictive_controller;
    
public:
    // 기존 생성자 사용
    EnhancedBatteryHeater(NTCSensor* sensor, gpio_num_t stepup_pin, 
                         gpio_num_t pwm_output_pin) 
        : BatteryHeater(sensor, stepup_pin, pwm_output_pin) {}
    
    void setControlMode(ControlMode mode) {
        current_mode = mode;
        if (mode == CONTROL_PID) {
            pid_controller.reset();
        }
        ESP_LOGI("ENHANCED_HEATER", "제어 모드 변경: %d", mode);
    }
    
    // 동적 제어 버전의 updateHeater
    bool updateHeaterDynamic(float external_temp = NAN) {
        float battery_temp;
        if (!ntc_sensor->readTemperature(&battery_temp)) {
            return false;
        }
        
        uint8_t dynamic_duty = 0;
        
        switch (current_mode) {
            case CONTROL_FIXED:
                return BatteryHeater::updateHeater(); // 기존 방식
                
            case CONTROL_PROPORTIONAL:
                dynamic_duty = p_controller.calculateDuty(battery_temp);
                break;
                
            case CONTROL_PID:
                dynamic_duty = pid_controller.calculateDuty(battery_temp);
                break;
                
            case CONTROL_ADAPTIVE:
                dynamic_duty = adaptive_controller.calculateDuty(battery_temp, external_temp);
                break;
                
            case CONTROL_PREDICTIVE:
                dynamic_duty = predictive_controller.calculateDuty(battery_temp);
                break;
        }
        
        // 동적 듀티 적용
        if (dynamic_duty > 0) {
            if (!stepup_enabled) {
                enableStepUpConverter();
            }
            setPWMDuty(dynamic_duty);
        } else {
            setPWMDuty(0);
            disableStepUpConverter();
        }
        
        ESP_LOGD("DYNAMIC", "배터리: %.2f°C → 동적 듀티: %d%%", 
                battery_temp, dynamic_duty);
        
        return true;
    }
};