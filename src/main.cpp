#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_chip_info.h"
#include "esp_flash.h"

#include "sensor_config.h"
#include "sensor_types.h"
#include "sensor_manager.h"

static const char *TAG = "MAIN";

// 측정 관련 변수
static uint32_t measurement_count = 0;

void printSystemInfo() {
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    uint32_t flash_size;
    esp_flash_get_size(NULL, &flash_size);
    
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    
    ESP_LOGI(TAG, "╔═══════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║                ESP32C6 시스템 정보                 ║");
    ESP_LOGI(TAG, "╠═══════════════════════════════════════════════════╣");
    ESP_LOGI(TAG, "║ 칩 모델     │ ESP32C6                          ║");
    ESP_LOGI(TAG, "║ 칩 리비전   │ v%d.%d                           ║", chip_info.revision / 100, chip_info.revision % 100);
    ESP_LOGI(TAG, "║ CPU 코어    │ %d개                             ║", chip_info.cores);
    ESP_LOGI(TAG, "║ 플래시 크기 │ %lu MB                           ║", flash_size / (1024 * 1024));
    ESP_LOGI(TAG, "║ 사용 가능한 │ %lu KB                           ║", esp_get_free_heap_size() / 1024);
    ESP_LOGI(TAG, "║ 힙 메모리   │                                   ║");
    ESP_LOGI(TAG, "║ MAC 주소    │ %02X:%02X:%02X:%02X:%02X:%02X    ║", 
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "║ I2C 핀 설정 │ SDA: GPIO%d, SCL: GPIO%d (HP Core) ║", I2C_SDA_PIN, I2C_SCL_PIN);
    ESP_LOGI(TAG, "║ LP I2C 핀   │ SDA: GPIO%d, SCL: GPIO%d (LP Core) ║", LP_I2C_SDA_PIN, LP_I2C_SCL_PIN);
    ESP_LOGI(TAG, "╚═══════════════════════════════════════════════════╝");
}

void printSensorData(const SensorData &data) {
    ESP_LOGI(TAG, "╔═══════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║                센서 데이터                         ║");
    ESP_LOGI(TAG, "╠═══════════════════════════════════════════════════╣");
    
    if (data.aht20_available) {
        ESP_LOGI(TAG, "║ AHT20       │ 온도: %6.2f°C  습도: %6.2f%% ║", 
                data.temperature_aht20, data.humidity_aht20);
    } else {
        ESP_LOGI(TAG, "║ AHT20       │ ❌ 오프라인                       ║");
    }
    
    if (data.bmp280_available) {
        ESP_LOGI(TAG, "║ BMP280      │ 온도: %6.2f°C  기압: %7.2fhPa ║", 
                data.temperature_bmp280, data.pressure_bmp280);
    } else {
        ESP_LOGI(TAG, "║ BMP280      │ ❌ 오프라인                       ║");
    }
    
    if (data.scd41_available) {
        ESP_LOGI(TAG, "║ SCD41       │ CO2: %5.0fppm  온도: %6.2f°C  ║", 
                data.co2_scd41, data.temperature_scd41);
        ESP_LOGI(TAG, "║             │ 습도: %6.2f%%                  ║", 
                data.humidity_scd41);
    } else {
        ESP_LOGI(TAG, "║ SCD41       │ ❌ 오프라인                       ║");
    }
    
    if (data.sgp40_available) {
        ESP_LOGI(TAG, "║ SGP40       │ VOC Index: %4.0f                   ║", 
                data.voc_index_sgp40);
    } else {
        ESP_LOGI(TAG, "║ SGP40       │ ❌ 오프라인                       ║");
    }
    
    ESP_LOGI(TAG, "║ 측정 시간   │ %llu초                            ║", data.timestamp / 1000);
    ESP_LOGI(TAG, "╚═══════════════════════════════════════════════════╝");
}

extern "C" void app_main() {
    // ESP-IDF 로깅 레벨 설정
    esp_log_level_set("*", ESP_LOG_DEBUG);
    esp_log_level_set("SENSOR_MGR", ESP_LOG_DEBUG);
    esp_log_level_set("AHT20", ESP_LOG_INFO);
    
    ESP_LOGI(TAG, "🚀 Xiao ESP32C6 센서 테스트 (DS3231 RTC + 적응형 절전 모드)");
    ESP_LOGI(TAG, "AHT20 + BMP280 + SCD41 + SGP40 + DS3231 + INA226 통합 시스템");
    
    // 시스템 정보 출력
    printSystemInfo();
    
    // 센서 매니저 생성
    ESP_LOGI(TAG, "센서 매니저 생성 중...");
    SensorManager sensorManager;
    
    // 센서 초기화 - 여기서 상세한 I2C 스캔이 실행됩니다
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "🔧 센서 초기화 시작...");
    ESP_LOGI(TAG, "════════════════════════════════════════");
    
    // I2C 버스 스캔 (프로토타입용 - 연결된 센서 확인)
    sensorManager.scanI2CBus();
    ESP_LOGI(TAG, "────────────────────────────────────────");
    
    bool init_success = sensorManager.init();
    
    // DS3231 RTC 초기화
    ESP_LOGI(TAG, "🕒 DS3231 RTC 초기화...");
    bool rtc_success = sensorManager.initRTC();
    if (rtc_success) {
        ESP_LOGI(TAG, "✅ DS3231 RTC 초기화 성공");
        
        // RTC 온도 센서 확인
        float rtc_temp = sensorManager.getRTCTemperature();
        ESP_LOGI(TAG, "DS3231 온도: %.2f°C", rtc_temp);
        
        // 시스템 시간 표시
        sensorManager.getSystemTime();
        
        // 시간이 설정되지 않은 경우 기본 시간 설정 (2024년 8월 22일 12:00:00)
        // 실제 운영에서는 NTP나 사용자 입력으로 설정해야 함
        // if (!sensorManager.getSystemTime()) {
        //     ESP_LOGI(TAG, "기본 시간 설정 중...");
        //     sensorManager.setSystemTime(2024, 8, 22, 12, 0, 0);
        // }
    } else {
        ESP_LOGW(TAG, "⚠️  DS3231 RTC 초기화 실패 - 내부 시계 사용");
    }
    
    ESP_LOGI(TAG, "════════════════════════════════════════");
    if (!init_success) {
        ESP_LOGE(TAG, "❌ 센서 초기화 실패!");
        ESP_LOGE(TAG, "SensorManager::init() 함수에서 false 반환됨");
        ESP_LOGE(TAG, "가능한 원인:");
        ESP_LOGE(TAG, "  1. I2C 드라이버 설치 실패");
        ESP_LOGE(TAG, "  2. I2C 버스에서 센서를 찾을 수 없음");
        ESP_LOGE(TAG, "  3. 하드웨어 연결 문제");
        
        // 초기화 실패해도 계속 진행 (디버깅 목적)
        ESP_LOGW(TAG, "초기화 실패했지만 계속 진행합니다 (디버깅 모드)");
    } else {
        ESP_LOGI(TAG, "✅ 센서 매니저 초기화 완료!");
        ESP_LOGI(TAG, "감지된 센서: %d개", sensorManager.getWorkingSensorCount());
        ESP_LOGI(TAG, "DS3231 RTC: %s", rtc_success ? "초기화됨" : "초기화 실패");
        ESP_LOGI(TAG, "현재 전력 모드: %s", 
                sensorManager.getCurrentPowerMode() == POWER_MODE_NORMAL ? "일반 모드 (HP Core)" : "저전력 모드 (LP Core)");
    }
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "📊 센서 데이터 측정 시작...");
    ESP_LOGI(TAG, "5초마다 측정합니다.");
    ESP_LOGI(TAG, "");
    
    // 메인 루프
    while (1) {
        measurement_count++;
        
        ESP_LOGI(TAG, "📈 측정 #%lu (작동 센서: %d개)", 
                measurement_count, sensorManager.getWorkingSensorCount());
        
        // 센서 데이터 읽기 - 이제 실제 센서에서 읽어옵니다
        SensorData sensor_data = sensorManager.readAllSensors();
        
        // 데이터 유효성 검증 (프로토타입용)
        sensorManager.validateSensorData(sensor_data);
        
        // 데이터 출력
        printSensorData(sensor_data);
        
        // 배터리 상태 읽기 및 출력
        BatteryStatus battery_status;
        if (sensorManager.getBatteryStatus(&battery_status)) {
            ESP_LOGI(TAG, "🔋 배터리 상태: %.3fV, %7.2fmA, %7.2fmW [%s]",
                     battery_status.voltage,
                     battery_status.current,
                     battery_status.power,
                     battery_status.is_charging ? "충전 중" : "방전 중");
        } else {
            ESP_LOGD(TAG, "배터리 상태를 읽을 수 없음 (INA226이 없거나 읽기 실패)");
        }
        
        // RTC 기반 시간 정보 및 브로드캐스트 메시지 확인
        if (rtc_success) {
            // RTC 알람 체크 (Wake-up 신호)
            if (sensorManager.isWakeupByAlarm()) {
                ESP_LOGI(TAG, "⏰ RTC 알람에 의한 Wake-up 감지");
            }
            
            // 브로드캐스트 메시지 체크 (10회마다)
            if (measurement_count % 10 == 1) {
                ESP_LOGI(TAG, "📡 브로드캐스트 메시지 확인...");
                if (sensorManager.checkBroadcastMessages()) {
                    ESP_LOGI(TAG, "🔔 리시버 노드로부터 모드 변경 신호 수신");
                }
            }
        }
        
        // 진단 (5회마다 한번씩 - 프로토타입 단계에서는 더 자주 확인)
        if (measurement_count % 5 == 1) {
            sensorManager.diagnoseSensors(sensor_data);
        }
        
        // 센서가 하나도 작동하지 않으면 복구 시도
        if (!sensorManager.hasWorkingSensors()) {
            ESP_LOGW(TAG, "⚠️  작동하는 센서가 없습니다. 복구를 시도합니다.");
            if (sensorManager.recoverFailedSensors()) {
                ESP_LOGI(TAG, "🎉 센서 복구 성공! 측정을 계속합니다.");
            } else {
                ESP_LOGE(TAG, "❌ 센서 복구 실패. 하드웨어 연결을 확인하세요.");
            }
        }
        
        // 연결 상태 체크 (20회마다)
        if (measurement_count % 20 == 0) {
            sensorManager.checkSensorConnections();
        }
        
        // 5초 대기
        vTaskDelay(pdMS_TO_TICKS(MEASUREMENT_INTERVAL_MS));
    }
}