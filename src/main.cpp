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
    
    ESP_LOGI(TAG, "측정 시간: %llu초", data.timestamp / 1000);
    ESP_LOGI(TAG, "╚═══════════════════════════════════════════════════╝");
}

extern "C" void app_main() {
    // ESP-IDF 로깅 레벨 설정
    esp_log_level_set("*", ESP_LOG_INFO);
    
    ESP_LOGI(TAG, "🚀 Xiao ESP32C6 센서 테스트 (모듈화 버전 - ESP-IDF)");
    ESP_LOGI(TAG, "기본 구조 구현 - 실제 센서 읽기는 다음 단계에서");
    
    // 시스템 정보 출력
    printSystemInfo();
    
    // 센서 매니저 생성 및 초기화
    sensorManager = new SensorManager();
    
    if (!sensorManager->init()) {
        ESP_LOGE(TAG, "❌ 센서 초기화 실패!");
    } else {
        ESP_LOGI(TAG, "✅ 센서 매니저 초기화 완료!");
        ESP_LOGI(TAG, "감지된 센서: %d개", sensorManager->getWorkingSensorCount());
    }
    
    ESP_LOGI(TAG, "📊 센서 데이터 측정 시작 (더미 데이터)...");
    
    // 메인 루프
    while (1) {
        measurement_count++;
        
        ESP_LOGI(TAG, "📈 측정 #%lu", measurement_count);
        
        // 센서 데이터 읽기 (현재는 더미 데이터)
        SensorData sensor_data = sensorManager->readAllSensors();
        
        // 데이터 출력
        printSensorData(sensor_data);
        
        // 진단
        sensorManager->diagnoseSensors(sensor_data);
        ESP_LOGI(TAG, "I2C 드라이버 상태 확인...");
        ESP_LOGI(TAG, "SDA: GPIO%d, SCL: GPIO%d", I2C_SDA_PIN, I2C_SCL_PIN);
        
        // 5초 대기
        vTaskDelay(pdMS_TO_TICKS(MEASUREMENT_INTERVAL_MS));
    }
}