#include "battery_heater.h"
#include "esp_timer.h"

const char* BatteryHeater::TAG = "BATTERY_HEATER";

BatteryHeater::BatteryHeater(NTCSensor* sensor, gpio_num_t stepup_pin, gpio_num_t pwm_output_pin,
                             ledc_channel_t channel, ledc_timer_t timer)
    : ntc_sensor(sensor), stepup_en_pin(stepup_pin), pwm_pin(pwm_output_pin),
      ledc_channel(channel), ledc_timer(timer),
      current_state(HEATER_UNKNOWN), previous_state(HEATER_UNKNOWN),
      stepup_enabled(false), initialized(false),
      last_battery_temp(-999.0f), last_update_ms(0), state_change_count(0),
      total_heating_time_ms(0), heating_cycles(0) {
}

BatteryHeater::~BatteryHeater() {
    deinit();
}

bool BatteryHeater::init() {
    ESP_LOGI(TAG, "배터리 히터 시스템 초기화 시작");
    
    if (!ntc_sensor || !ntc_sensor->isInitialized()) {
        ESP_LOGE(TAG, "NTC 센서가 초기화되지 않음");
        return false;
    }
    
    // 스텝업 컨버터 EN 핀 초기화 (LOW = 비활성화)
    gpio_config_t stepup_config = {
        .pin_bit_mask = (1ULL << stepup_en_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    esp_err_t ret = gpio_config(&stepup_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "스텝업 컨버터 EN 핀 초기화 실패: %s", esp_err_to_name(ret));
        return false;
    }
    
    gpio_set_level(stepup_en_pin, 0);  // 초기에는 비활성화
    
    // PWM 초기화
    if (!initPWM()) {
        ESP_LOGE(TAG, "PWM 초기화 실패");
        return false;
    }
    
    // 초기 온도 측정
    float initial_temp;
    if (ntc_sensor->readTemperature(&initial_temp)) {
        last_battery_temp = initial_temp;
        ESP_LOGI(TAG, "초기 배터리 온도: %.2f°C", initial_temp);
        
        // 초기 상태 결정
        current_state = calculateNewState(initial_temp);
        updateHeaterOutput(current_state);
        
        ESP_LOGI(TAG, "✅ 배터리 히터 초기화 완료 - 초기 상태: %s", 
                getStateName(current_state));
        
        initialized = true;
        last_update_ms = esp_timer_get_time() / 1000;
        return true;
    } else {
        ESP_LOGE(TAG, "❌ 초기 온도 측정 실패");
        return false;
    }
}

bool BatteryHeater::initPWM() {
    // LEDC 타이머 설정
    ledc_timer_config_t ledc_timer_config = {
        .speed_mode = ledc_mode,
        .duty_resolution = LEDC_TIMER_8_BIT,  // 8비트 (0-255)
        .timer_num = ledc_timer,
        .freq_hz = HEATER_PWM_FREQ_HZ,        // 1kHz
        .clk_cfg = LEDC_AUTO_CLK
    };
    
    esp_err_t ret = ledc_timer_config(&ledc_timer_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC 타이머 설정 실패: %s", esp_err_to_name(ret));
        return false;
    }
    
    // LEDC 채널 설정
    ledc_channel_config_t ledc_channel_config = {
        .gpio_num = pwm_pin,
        .speed_mode = ledc_mode,
        .channel = ledc_channel,
        .timer_sel = ledc_timer,
        .duty = 0,                           // 초기 듀티 0%
        .hpoint = 0
    };
    
    ret = ledc_channel_config(&ledc_channel_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC 채널 설정 실패: %s", esp_err_to_name(ret));
        return false;
    }
    
    ESP_LOGI(TAG, "PWM 초기화 완료 (핀: %d, 주파수: %dHz)", pwm_pin, HEATER_PWM_FREQ_HZ);
    return true;
}

bool BatteryHeater::enableStepUpConverter() {
    if (!stepup_enabled) {
        gpio_set_level(stepup_en_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(50));  // 스텝업 컨버터 안정화 대기
        stepup_enabled = true;
        ESP_LOGD(TAG, "스텝업 컨버터 활성화");
    }
    return true;
}

void BatteryHeater::disableStepUpConverter() {
    if (stepup_enabled) {
        setPWMDuty(0);  // PWM 먼저 0으로
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(stepup_en_pin, 0);
        stepup_enabled = false;
        ESP_LOGD(TAG, "스텝업 컨버터 비활성화");
    }
}

void BatteryHeater::setPWMDuty(uint8_t duty_percent) {
    if (duty_percent > 100) duty_percent = 100;
    
    uint32_t duty_value = (duty_percent * 255) / 100;  // 8비트 듀티 계산
    
    esp_err_t ret = ledc_set_duty(ledc_mode, ledc_channel, duty_value);
    if (ret == ESP_OK) {
        ledc_update_duty(ledc_mode, ledc_channel);
        ESP_LOGV(TAG, "PWM 듀티 설정: %d%% (값: %lu)", duty_percent, duty_value);
    } else {
        ESP_LOGW(TAG, "PWM 듀티 설정 실패: %s", esp_err_to_name(ret));
    }
}

heater_state_t BatteryHeater::calculateNewState(float battery_temp) {
    // 히스테리시스 제어 로직 (sensor_config.h의 설계 구현)
    
    if (battery_temp < -2.0f) {
        return HEATER_HIGH;  // 급속 가열 (75% PWM)
    }
    else if (battery_temp < 0.0f) {
        return HEATER_MED;   // 표준 가열 (50% PWM)
    }
    else if (battery_temp < 3.0f) {
        return HEATER_LOW;   // 유지 가열 (25% PWM)
    }
    else if (battery_temp < 6.0f) {
        // 히스테리시스 구간: 이전 상태 유지
        if (current_state == HEATER_OFF) {
            return HEATER_OFF;  // 이미 꺼져있으면 계속 OFF
        } else {
            return current_state;  // 현재 상태 유지
        }
    }
    else if (battery_temp >= 8.0f) {
        return HEATER_OFF;   // 완전 꺼짐 (0% PWM)
    }
    else {
        // 6°C ~ 8°C: 점진적 감소
        if (current_state == HEATER_HIGH || current_state == HEATER_MED) {
            return HEATER_LOW;  // 고/중출력에서 저출력으로
        } else {
            return current_state;  // 현재 상태 유지
        }
    }
}

void BatteryHeater::updateHeaterOutput(heater_state_t new_state) {
    if (new_state == current_state) {
        return;  // 상태 변경 없음
    }
    
    previous_state = current_state;
    current_state = new_state;
    state_change_count++;
    
    uint8_t duty = getStatePWMDuty(new_state);
    
    if (duty > 0) {
        // 히터 켜기
        if (!stepup_enabled) {
            enableStepUpConverter();
        }
        setPWMDuty(duty);
        
        if (previous_state == HEATER_OFF) {
            heating_cycles++;
            ESP_LOGI(TAG, "🔥 히터 켜짐: %s (%.1fW) - 사이클: %lu", 
                    getStateName(new_state), getStatePowerConsumption(new_state), heating_cycles);
        } else {
            ESP_LOGI(TAG, "⚡ 히터 출력 변경: %s -> %s (%.1fW)", 
                    getStateName(previous_state), getStateName(new_state), 
                    getStatePowerConsumption(new_state));
        }
    } else {
        // 히터 끄기
        setPWMDuty(0);
        disableStepUpConverter();
        ESP_LOGI(TAG, "❄️ 히터 꺼짐: %s -> %s", 
                getStateName(previous_state), getStateName(new_state));
    }
}

uint8_t BatteryHeater::getStatePWMDuty(heater_state_t state) {
    switch (state) {
        case HEATER_OFF:  return 0;
        case HEATER_LOW:  return 25;   // 25% -> 0.39W
        case HEATER_MED:  return 50;   // 50% -> 0.625W
        case HEATER_HIGH: return 75;   // 75% -> 1.41W
        case HEATER_MAX:  return 100;  // 100% -> 2.5W
        default:          return 0;
    }
}

const char* BatteryHeater::getStateName(heater_state_t state) {
    switch (state) {
        case HEATER_OFF:     return "OFF";
        case HEATER_LOW:     return "LOW(25%)";
        case HEATER_MED:     return "MED(50%)";
        case HEATER_HIGH:    return "HIGH(75%)";
        case HEATER_MAX:     return "MAX(100%)";
        case HEATER_UNKNOWN: return "UNKNOWN";
        default:             return "INVALID";
    }
}

float BatteryHeater::getStatePowerConsumption(heater_state_t state) {
    switch (state) {
        case HEATER_OFF:  return 0.0f;
        case HEATER_LOW:  return PWM_25_POWER_W;   // 0.39W
        case HEATER_MED:  return PWM_50_POWER_W;   // 0.625W
        case HEATER_HIGH: return PWM_75_POWER_W;   // 1.41W
        case HEATER_MAX:  return PWM_100_POWER_W;  // 2.5W
        default:          return 0.0f;
    }
}

bool BatteryHeater::updateHeater() {
    if (!initialized) {
        ESP_LOGE(TAG, "히터가 초기화되지 않음");
        return false;
    }
    
    float battery_temp;
    if (!ntc_sensor->readTemperature(&battery_temp)) {
        ESP_LOGW(TAG, "배터리 온도 읽기 실패");
        return false;
    }
    
    // 온도 변화량 체크
    float temp_diff = abs(battery_temp - last_battery_temp);
    if (temp_diff > 10.0f) {
        ESP_LOGW(TAG, "급격한 온도 변화 감지: %.2f°C -> %.2f°C (차이: %.2f°C)", 
                last_battery_temp, battery_temp, temp_diff);
    }
    
    last_battery_temp = battery_temp;
    
    // 새로운 상태 계산 및 적용
    heater_state_t new_state = calculateNewState(battery_temp);
    updateHeaterOutput(new_state);
    
    // 통계 업데이트
    uint32_t current_ms = esp_timer_get_time() / 1000;
    if (last_update_ms > 0 && current_state != HEATER_OFF) {
        total_heating_time_ms += (current_ms - last_update_ms);
    }
    last_update_ms = current_ms;
    
    ESP_LOGD(TAG, "히터 업데이트: %.2f°C -> %s", battery_temp, getStateName(current_state));
    return true;
}

void BatteryHeater::emergencyShutdown() {
    ESP_LOGW(TAG, "🚨 비상 히터 정지 실행");
    
    setPWMDuty(0);
    disableStepUpConverter();
    current_state = HEATER_OFF;
    
    ESP_LOGI(TAG, "비상 정지 완료");
}

float BatteryHeater::getCurrentPowerConsumption() const {
    if (stepup_enabled) {
        return getStatePowerConsumption(current_state);
    }
    return 0.0f;
}

void BatteryHeater::printStatus() {
    ESP_LOGI(TAG, "=== 배터리 히터 상태 ===");
    ESP_LOGI(TAG, "배터리 온도: %.2f°C", last_battery_temp);
    ESP_LOGI(TAG, "현재 상태: %s", getStateName(current_state));
    ESP_LOGI(TAG, "스텝업 컨버터: %s", stepup_enabled ? "활성화" : "비활성화");
    ESP_LOGI(TAG, "현재 소모전력: %.2fW", getCurrentPowerConsumption());
    ESP_LOGI(TAG, "PWM 듀티: %d%%", getStatePWMDuty(current_state));
}

void BatteryHeater::printStatistics() {
    ESP_LOGI(TAG, "=== 히터 통계 ===");
    ESP_LOGI(TAG, "총 가열 시간: %lu초", total_heating_time_ms / 1000);
    ESP_LOGI(TAG, "가열 사이클: %lu회", heating_cycles);
    ESP_LOGI(TAG, "상태 변경: %lu회", state_change_count);
    
    if (heating_cycles > 0) {
        ESP_LOGI(TAG, "평균 가열 시간: %lu초/사이클", 
                (total_heating_time_ms / 1000) / heating_cycles);
    }
}

void BatteryHeater::resetStatistics() {
    total_heating_time_ms = 0;
    heating_cycles = 0;
    state_change_count = 0;
    ESP_LOGI(TAG, "통계 초기화 완료");
}

void BatteryHeater::deinit() {
    if (initialized) {
        emergencyShutdown();
        
        // PWM 채널 정지
        ledc_stop(ledc_mode, ledc_channel, 0);
        
        initialized = false;
        ESP_LOGI(TAG, "배터리 히터 정리 완료");
    }
}