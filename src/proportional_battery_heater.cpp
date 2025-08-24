#include "proportional_battery_heater.h"
#include "esp_timer.h"
#include <cstring>

const char* ProportionalBatteryHeater::TAG = "PROP_HEATER";

ProportionalBatteryHeater::ProportionalBatteryHeater(NTCSensor* sensor, 
                                                     gpio_num_t stepup_pin, 
                                                     gpio_num_t pwm_output_pin)
    : BatteryHeater(sensor, stepup_pin, pwm_output_pin) {
}

bool ProportionalBatteryHeater::updateProportionalHeater() {
    if (!initialized) {
        ESP_LOGE(TAG, "히터가 초기화되지 않음");
        return false;
    }
    
    float battery_temp;
    if (!ntc_sensor->readTemperature(&battery_temp)) {
        ESP_LOGW(TAG, "배터리 온도 읽기 실패");
        return false;
    }
    
    last_battery_temp = battery_temp;
    
    if (proportional_mode_enabled) {
        // 비례 제어 모드
        uint8_t proportional_duty = calculateProportionalDuty(battery_temp);
        
        if (proportional_duty > 0) {
            // 히터 켜기
            if (!stepup_enabled) {
                enableStepUpConverter();
                if (previous_state == HEATER_OFF) {
                    heating_cycles++;
                    ESP_LOGI(TAG, "🔥 비례제어 히터 시작: %.1f°C → %d%% (사이클: %lu)", 
                            battery_temp, proportional_duty, heating_cycles);
                }
            }
            setPWMDuty(proportional_duty);
            current_state = (proportional_duty > 75) ? HEATER_HIGH :
                           (proportional_duty > 50) ? HEATER_MED :
                           (proportional_duty > 25) ? HEATER_LOW : HEATER_OFF;
        } else {
            // 히터 끄기
            if (stepup_enabled) {
                setPWMDuty(0);
                disableStepUpConverter();
                ESP_LOGI(TAG, "❄️ 비례제어 히터 정지: %.1f°C", battery_temp);
            }
            current_state = HEATER_OFF;
        }
        
        ESP_LOGD(TAG, "비례제어: %.2f°C → %d%% (목표: %.1f°C, 오차: %.2f°C)", 
                battery_temp, proportional_duty, target_temperature, 
                target_temperature - battery_temp);
    } else {
        // 기존 히스테리시스 모드로 폴백
        return BatteryHeater::updateHeater();
    }
    
    // 통계 업데이트
    uint32_t current_ms = esp_timer_get_time() / 1000;
    if (last_update_ms > 0 && current_state != HEATER_OFF) {
        total_heating_time_ms += (current_ms - last_update_ms);
    }
    last_update_ms = current_ms;
    
    return true;
}

uint8_t ProportionalBatteryHeater::calculateProportionalDuty(float current_temp) {
    // 온도 오차 계산
    float error = target_temperature - current_temp;
    
    // 데드존 적용 (작은 오차 무시)
    if (abs(error) < dead_zone) {
        error = 0.0f;
    }
    
    // 비례 제어: PWM = Kp × error
    float duty_float = proportional_gain * error;
    
    // 듀티 제한
    if (duty_float < min_duty) duty_float = min_duty;
    if (duty_float > max_duty) duty_float = max_duty;
    
    // 통계 업데이트
    updateStatistics(error);
    
    uint8_t duty = (uint8_t)duty_float;
    
    ESP_LOGV(TAG, "비례계산: 오차=%.2f°C, Kp=%.1f, 듀티=%.1f%% → %d%%", 
            error, proportional_gain, duty_float, duty);
    
    return duty;
}

void ProportionalBatteryHeater::updateStatistics(float error) {
    control_updates++;
    
    // 평균 오차 업데이트 (이동평균)
    avg_error = (avg_error * (control_updates - 1) + abs(error)) / control_updates;
    
    // 최대 오차 업데이트
    if (abs(error) > max_error) {
        max_error = abs(error);
    }
}

void ProportionalBatteryHeater::printControlParameters() {
    ESP_LOGI(TAG, "=== 비례 제어 파라미터 ===");
    ESP_LOGI(TAG, "목표 온도: %.1f°C", target_temperature);
    ESP_LOGI(TAG, "비례 게인 (Kp): %.1f", proportional_gain);
    ESP_LOGI(TAG, "듀티 범위: %.1f%% ~ %.1f%%", min_duty, max_duty);
    ESP_LOGI(TAG, "데드존: ±%.1f°C", dead_zone);
    ESP_LOGI(TAG, "비례 모드: %s", proportional_mode_enabled ? "활성화" : "비활성화");
}

void ProportionalBatteryHeater::printPerformanceStatistics() {
    ESP_LOGI(TAG, "=== 비례 제어 성능 통계 ===");
    ESP_LOGI(TAG, "제어 업데이트: %lu회", control_updates);
    ESP_LOGI(TAG, "평균 오차: %.2f°C", avg_error);
    ESP_LOGI(TAG, "최대 오차: %.2f°C", max_error);
    ESP_LOGI(TAG, "총 가열 시간: %lu초", total_heating_time_ms / 1000);
    ESP_LOGI(TAG, "가열 사이클: %lu회", heating_cycles);
    
    if (heating_cycles > 0) {
        ESP_LOGI(TAG, "평균 가열시간/사이클: %lu초", 
                (total_heating_time_ms / 1000) / heating_cycles);
    }
}

void ProportionalBatteryHeater::resetStatistics() {
    avg_error = 0.0f;
    max_error = 0.0f;
    control_updates = 0;
    total_heating_time_ms = 0;
    heating_cycles = 0;
    ESP_LOGI(TAG, "성능 통계 초기화 완료");
}

void ProportionalBatteryHeater::handleTuningCommand(const char* command) {
    // 실시간 튜닝 명령어 처리
    // 예: "kp=20", "target=6", "dead=0.3", "max=80"
    
    if (strncmp(command, "kp=", 3) == 0) {
        float new_kp = atof(command + 3);
        if (new_kp >= 1.0f && new_kp <= 50.0f) {
            proportional_gain = new_kp;
            ESP_LOGI(TAG, "비례 게인 변경: Kp=%.1f", new_kp);
        } else {
            ESP_LOGW(TAG, "잘못된 Kp 값: %.1f (범위: 1.0~50.0)", new_kp);
        }
    }
    else if (strncmp(command, "target=", 7) == 0) {
        float new_target = atof(command + 7);
        if (new_target >= -10.0f && new_target <= 20.0f) {
            target_temperature = new_target;
            ESP_LOGI(TAG, "목표 온도 변경: %.1f°C", new_target);
        } else {
            ESP_LOGW(TAG, "잘못된 목표 온도: %.1f°C (범위: -10~20°C)", new_target);
        }
    }
    else if (strncmp(command, "dead=", 5) == 0) {
        float new_dead = atof(command + 5);
        if (new_dead >= 0.0f && new_dead <= 2.0f) {
            dead_zone = new_dead;
            ESP_LOGI(TAG, "데드존 변경: ±%.1f°C", new_dead);
        } else {
            ESP_LOGW(TAG, "잘못된 데드존 값: %.1f°C (범위: 0~2°C)", new_dead);
        }
    }
    else if (strncmp(command, "max=", 4) == 0) {
        float new_max = atof(command + 4);
        if (new_max >= 10.0f && new_max <= 100.0f) {
            max_duty = new_max;
            ESP_LOGI(TAG, "최대 듀티 변경: %.1f%%", new_max);
        } else {
            ESP_LOGW(TAG, "잘못된 최대 듀티: %.1f%% (범위: 10~100%%)", new_max);
        }
    }
    else if (strcmp(command, "reset") == 0) {
        resetStatistics();
    }
    else if (strcmp(command, "params") == 0) {
        printControlParameters();
    }
    else if (strcmp(command, "stats") == 0) {
        printPerformanceStatistics();
    }
    else if (strcmp(command, "enable") == 0) {
        proportional_mode_enabled = true;
        ESP_LOGI(TAG, "비례 제어 활성화");
    }
    else if (strcmp(command, "disable") == 0) {
        proportional_mode_enabled = false;
        ESP_LOGI(TAG, "비례 제어 비활성화 (히스테리시스 모드)");
    }
    else {
        ESP_LOGW(TAG, "알 수 없는 명령어: %s", command);
        ESP_LOGI(TAG, "사용 가능한 명령어:");
        ESP_LOGI(TAG, "  kp=<값>     : 비례 게인 설정 (1.0~50.0)");
        ESP_LOGI(TAG, "  target=<값> : 목표 온도 설정 (-10~20°C)");
        ESP_LOGI(TAG, "  dead=<값>   : 데드존 설정 (0~2°C)");
        ESP_LOGI(TAG, "  max=<값>    : 최대 듀티 설정 (10~100%)");
        ESP_LOGI(TAG, "  enable      : 비례 제어 활성화");
        ESP_LOGI(TAG, "  disable     : 비례 제어 비활성화");
        ESP_LOGI(TAG, "  params      : 현재 파라미터 출력");
        ESP_LOGI(TAG, "  stats       : 성능 통계 출력");
        ESP_LOGI(TAG, "  reset       : 통계 초기화");
    }
}