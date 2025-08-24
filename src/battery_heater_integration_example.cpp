// 배터리 히터 시스템 메인 루프 통합 예시 코드
// 실제 main.cpp에 추가할 코드 조각들

// 1. 헤더 추가 (main.cpp 상단에 추가)
#include "ntc_sensor.h"
#include "battery_heater.h"

// 2. 전역 변수 (main.cpp 전역 영역에 추가)
static uint32_t battery_heater_update_count = 0;
static uint32_t last_heater_statistics_print = 0;

// 3. 시스템 정보 출력 함수에 히터 핀 정보 추가
void printSystemInfoWithHeater() {
    // 기존 printSystemInfo() 내용 + 아래 추가
    ESP_LOGI(TAG, "║ 배터리 히터 │ NTC: GPIO%d, PWM: GPIO%d, EN: GPIO%d ║", 
             BATTERY_NTC_POWER_PIN, BATTERY_HEATER_PWM_PIN, STEPUP_CONVERTER_EN_PIN);
}

// 4. 센서 데이터 출력 함수에 배터리 히터 정보 추가
void printSensorDataWithHeater(const SensorData &data, SensorManager &sensorManager) {
    // 기존 printSensorData() 내용 실행
    printSensorData(data);
    
    // 배터리 히터 상태 출력 추가
    if (sensorManager.getHeaterState() != HEATER_UNKNOWN) {
        float battery_temp = sensorManager.getBatteryTemperature();
        heater_state_t heater_state = sensorManager.getHeaterState();
        
        ESP_LOGI(TAG, "║ 배터리 히터 │ 온도: %6.2f°C  상태: %-10s ║", 
                battery_temp, getHeaterStateName(heater_state));
    } else {
        ESP_LOGI(TAG, "║ 배터리 히터 │ ❌ 오프라인                       ║");
    }
}

// 5. 메인 루프에 추가할 배터리 히터 제어 코드
void batteryHeaterMainLoopIntegration(SensorManager &sensorManager, uint32_t measurement_count) {
    // 배터리 히터 업데이트 (매 측정마다 실행)
    if (sensorManager.updateBatteryHeater()) {
        battery_heater_update_count++;
        ESP_LOGD(TAG, "배터리 히터 업데이트 완료 (%lu회)", battery_heater_update_count);
    } else {
        ESP_LOGW(TAG, "배터리 히터 업데이트 실패");
    }
    
    // 히터 상태 출력 (10회마다)
    if (measurement_count % 10 == 1) {
        sensorManager.printHeaterStatus();
    }
    
    // 히터 통계 출력 (100회마다)
    if (measurement_count % 100 == 1) {
        sensorManager.printHeaterStatistics();
        last_heater_statistics_print = measurement_count;
    }
    
    // 비상 상황 체크 (극한 온도)
    float battery_temp = sensorManager.getBatteryTemperature();
    if (battery_temp < -30.0f || battery_temp > 60.0f) {
        ESP_LOGE(TAG, "🚨 배터리 온도 위험: %.2f°C - 비상 정지", battery_temp);
        sensorManager.emergencyHeaterShutdown();
    }
}

// 6. 실제 main.cpp app_main() 함수에 통합하는 방법
void integratedAppMain() {
    // ... 기존 초기화 코드 ...
    
    // 센서 매니저 생성 및 초기화
    SensorManager sensorManager;
    bool init_success = sensorManager.init();
    
    // 배터리 히터 초기화 추가
    bool heater_init_success = false;
    if (init_success) {
        ESP_LOGI(TAG, "🔥 배터리 히터 시스템 초기화...");
        heater_init_success = sensorManager.initBatteryHeater();
        if (heater_init_success) {
            ESP_LOGI(TAG, "✅ 배터리 히터 시스템 초기화 완료");
        } else {
            ESP_LOGW(TAG, "⚠️ 배터리 히터 초기화 실패 - 히터 없이 계속 진행");
        }
    }
    
    // ... 기존 RTC 초기화 코드 ...
    
    // 메인 루프
    while (1) {
        measurement_count++;
        
        // 기존 센서 측정 코드
        SensorData sensor_data = sensorManager.readAllSensors();
        
        // 배터리 히터 제어 (초기화 성공한 경우만)
        if (heater_init_success) {
            batteryHeaterMainLoopIntegration(sensorManager, measurement_count);
        }
        
        // 확장된 데이터 출력
        printSensorDataWithHeater(sensor_data, sensorManager);
        
        // ... 기존 나머지 코드 ...
        
        vTaskDelay(pdMS_TO_TICKS(MEASUREMENT_INTERVAL_MS));
    }
}

// 7. 히터 상태 이름 반환 함수 (유틸리티)
const char* getHeaterStateName(heater_state_t state) {
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

// 8. 개별 테스트 함수 (디버깅용)
void testBatteryHeaterSystem() {
    ESP_LOGI(TAG, "=== 배터리 히터 시스템 개별 테스트 ===");
    
    // NTC 센서 단독 테스트
    NTCSensor ntc_sensor(BATTERY_NTC_ADC_CHANNEL, BATTERY_NTC_POWER_PIN);
    if (ntc_sensor.init()) {
        float temp;
        if (ntc_sensor.readTemperature(&temp)) {
            ESP_LOGI(TAG, "NTC 센서 테스트 성공: %.2f°C", temp);
        }
        ntc_sensor.printCalibrationInfo();
    }
    
    // 배터리 히터 단독 테스트
    BatteryHeater heater(&ntc_sensor, STEPUP_CONVERTER_EN_PIN, 
                        BATTERY_HEATER_PWM_PIN);
    if (heater.init()) {
        ESP_LOGI(TAG, "배터리 히터 초기화 성공");
        
        // 5회 업데이트 테스트
        for (int i = 0; i < 5; i++) {
            heater.updateHeater();
            heater.printStatus();
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
        
        heater.printStatistics();
    }
}