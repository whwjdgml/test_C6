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

// 전역 객체 (C++ 스타일)
static SensorManager* sensorManager = nullptr;

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
    ESP_LOGI(TAG, "║ I2C 핀 설정 │ SDA: GPIO%d, SCL: GPIO%d         ║", I2C_SDA_PIN, I2C_SCL_PIN);
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
    
    ESP_LOGI(TAG, "║ 측정 시간   │ %llu초                            ║", data.timestamp / 1000);
    ESP_LOGI(TAG, "╚═══════════════════════════════════════════════════╝");
}

extern "C" void app_main() {
    // ESP-IDF 로깅 레벨 설정
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("SENSOR_MGR", ESP_LOG_INFO);
    esp_log_level_set("AHT20", ESP_LOG_INFO);
    
    ESP_LOGI(TAG, "🚀 Xiao ESP32C6 센서 테스트 (실제 센서 읽기 버전)");
    ESP_LOGI(TAG, "AHT20 + BMP280 센서 모듈 테스트");
    
    // 시스템 정보 출력
    printSystemInfo();
    
    // 센서 매니저 생성
    ESP_LOGI(TAG, "센서 매니저 생성 중...");
    sensorManager = new SensorManager();
    
    // 센서 초기화 - 여기서 상세한 I2C 스캔이 실행됩니다
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "🔧 센서 초기화 시작...");
    ESP_LOGI(TAG, "════════════════════════════════════════");
    
    bool init_success = sensorManager->init();
    
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
        ESP_LOGI(TAG, "감지된 센서: %d개", sensorManager->getWorkingSensorCount());
    }
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "📊 센서 데이터 측정 시작...");
    ESP_LOGI(TAG, "5초마다 측정합니다.");
    ESP_LOGI(TAG, "");
    
    // 메인 루프
    while (1) {
        measurement_count++;
        
        ESP_LOGI(TAG, "📈 측정 #%lu (작동 센서: %d개)", 
                measurement_count, sensorManager->getWorkingSensorCount());
        
        // 센서 데이터 읽기 - 이제 실제 센서에서 읽어옵니다
        SensorData sensor_data = sensorManager->readAllSensors();
        
        // 데이터 출력
        printSensorData(sensor_data);
        
        // 진단 (10회마다 한번씩만)
        if (measurement_count % 10 == 1) {
            sensorManager->diagnoseSensors(sensor_data);
        }
        
        // 센서가 하나도 작동하지 않으면 경고
        if (!sensorManager->hasWorkingSensors()) {
            ESP_LOGW(TAG, "⚠️  작동하는 센서가 없습니다. 하드웨어 연결을 확인하세요.");
        }
        
        // 5초 대기
        vTaskDelay(pdMS_TO_TICKS(MEASUREMENT_INTERVAL_MS));
    }
}