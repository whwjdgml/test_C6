#include "sensor_manager.h"
#include "sensor_config.h"
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_err.h"

static const char *TAG = "SENSOR_MGR";

// I2C 설정
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_TIMEOUT_MS   1000

// 생성자 구현
SensorManager::SensorManager() : aht20_initialized(false), bmp280_initialized(false) {
    // 기본 초기화
}

bool SensorManager::init() {
    ESP_LOGI(TAG, "SensorManager 초기화 중...");
    
    // I2C 초기화
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = 100000,
        },
        .clk_flags = 0,
    };
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C 파라미터 설정 실패: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C 드라이버 설치 실패: %s", esp_err_to_name(ret));
        return false;
    }
    
    ESP_LOGI(TAG, "I2C 초기화 완료 - SDA: GPIO%d, SCL: GPIO%d", I2C_SDA_PIN, I2C_SCL_PIN);
    
    // 센서 스캔 (더 상세한 로그)
    ESP_LOGI(TAG, "I2C 센서 스캔 시작 (0x01~0x7F)...");
    int devices_found = 0;
    
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "센서 발견: 0x%02X", addr);
            devices_found++;
            
            // 알려진 센서 타입 식별
            switch (addr) {
                case 0x38:
                    aht20_initialized = true;
                    ESP_LOGI(TAG, "  -> AHT20/AHT21 센서");
                    break;
                case 0x76:
                case 0x77:
                    bmp280_initialized = true;
                    ESP_LOGI(TAG, "  -> BMP280/BME280 센서");
                    break;
                case 0x44:
                    ESP_LOGI(TAG, "  -> SHT30/SHT31 센서");
                    break;
                case 0x5A:
                    ESP_LOGI(TAG, "  -> MLX90614 온도센서");
                    break;
                default:
                    ESP_LOGI(TAG, "  -> 미확인 디바이스 (0x%02X)", addr);
                    break;
            }
        } else if (ret == ESP_ERR_TIMEOUT) {
            // 타임아웃은 정상 - 센서가 없는 주소
            continue;
        } else {
            // 다른 에러는 로그에 기록
            if (addr == 0x38 || addr == 0x76 || addr == 0x77) {
                ESP_LOGW(TAG, "주소 0x%02X 스캔 에러: %s", addr, esp_err_to_name(ret));
            }
        }
    }
    
    if (devices_found == 0) {
        ESP_LOGW(TAG, "I2C 디바이스를 찾을 수 없습니다!");
        ESP_LOGW(TAG, "다음을 확인하세요:");
        ESP_LOGW(TAG, "  1. 센서 전원 (VCC: 3.3V, GND 연결)");
        ESP_LOGW(TAG, "  2. I2C 연결 (SDA: D4/GPIO4, SCL: D5/GPIO5)");
        ESP_LOGW(TAG, "  3. 점퍼선 연결 상태");
        ESP_LOGW(TAG, "  4. 센서 모듈 불량 여부");
    } else {
        ESP_LOGI(TAG, "총 %d개의 I2C 디바이스 발견", devices_found);
    }
    
    return (aht20_initialized || bmp280_initialized);
}

SensorData SensorManager::readAllSensors() {
    SensorData data = {};  // 모든 필드를 0으로 초기화
    data.timestamp = esp_timer_get_time() / 1000;  // microseconds to milliseconds
    data.aht20_available = aht20_initialized;
    data.bmp280_available = bmp280_initialized;
    
    if (aht20_initialized) {
        // 임시 더미 데이터 (랜덤)
        data.temperature_aht20 = 23.5 + ((esp_random() % 20) - 10) / 10.0;
        data.humidity_aht20 = 65.0 + ((esp_random() % 100) - 50) / 10.0;
    }
    
    if (bmp280_initialized) {
        // 임시 더미 데이터 (랜덤)
        data.temperature_bmp280 = 23.3 + ((esp_random() % 20) - 10) / 10.0;
        data.pressure_bmp280 = 1013.25 + ((esp_random() % 40) - 20) / 10.0;
    }
    
    return data;
}

bool SensorManager::hasWorkingSensors() const {
    return (aht20_initialized || bmp280_initialized);
}

uint8_t SensorManager::getWorkingSensorCount() const {
    uint8_t count = 0;
    if (aht20_initialized) count++;
    if (bmp280_initialized) count++;
    return count;
}

void SensorManager::diagnoseSensors(const SensorData &data) {
    ESP_LOGI(TAG, "센서 진단:");
    ESP_LOGI(TAG, "  AHT20: %s", data.aht20_available ? "정상" : "오프라인");
    ESP_LOGI(TAG, "  BMP280: %s", data.bmp280_available ? "정상" : "오프라인");
    ESP_LOGI(TAG, "  작동 센서: %d개", this->getWorkingSensorCount());
}