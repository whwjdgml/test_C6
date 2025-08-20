#include "sensor_manager.h"
#include "sensor_config.h"
#include "aht20_sensor.h"
#include "bmp280_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "SENSOR_MGR";

// I2C 설정
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_TIMEOUT_MS   1000

// 생성자 구현
SensorManager::SensorManager() : aht20_sensor(nullptr), bmp280_sensor(nullptr), aht20_initialized(false), bmp280_initialized(false) {
    // 센서 객체 생성
    aht20_sensor = new AHT20Sensor();
    bmp280_sensor = new BMP280Sensor();
}

SensorManager::~SensorManager() {
    if (aht20_sensor) {
        delete aht20_sensor;
    }
    if (bmp280_sensor) {
        delete bmp280_sensor;
    }
}

bool SensorManager::init() {
    ESP_LOGI(TAG, "SensorManager 초기화 중...");
    ESP_LOGI(TAG, "설정된 I2C 핀: SDA=GPIO%d, SCL=GPIO%d", I2C_SDA_PIN, I2C_SCL_PIN);
    
    // 기존 I2C 드라이버가 있다면 제거
    ESP_LOGI(TAG, "기존 I2C 드라이버 정리 중...");
    i2c_driver_delete(I2C_MASTER_NUM);
    vTaskDelay(pdMS_TO_TICKS(100)); // 100ms 대기
    
    // I2C 설정
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_PIN;          // GPIO22
    conf.scl_io_num = I2C_SCL_PIN;          // GPIO23
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;         // 100kHz
    conf.clk_flags = 0;
    
    ESP_LOGI(TAG, "I2C 파라미터 설정 중... SDA=%d, SCL=%d", conf.sda_io_num, conf.scl_io_num);
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C 파라미터 설정 실패: %s", esp_err_to_name(ret));
        return false;
    }
    
    ESP_LOGI(TAG, "I2C 드라이버 설치 중...");
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C 드라이버 설치 실패: %s", esp_err_to_name(ret));
        return false;
    }
    
    ESP_LOGI(TAG, "I2C 초기화 완료");
    
    // 센서 초기화 대기
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // AHT20 센서 초기화 시도
    ESP_LOGI(TAG, "AHT20 센서 초기화 시도...");
    if (aht20_sensor && aht20_sensor->init()) {
        aht20_initialized = true;
        ESP_LOGI(TAG, "AHT20 센서 초기화 성공");
    } else {
        ESP_LOGW(TAG, "AHT20 센서 초기화 실패");
    }
    
    // BMP280 센서 초기화 시도
    ESP_LOGI(TAG, "BMP280 센서 초기화 시도...");
    if (bmp280_sensor && bmp280_sensor->init()) {
        bmp280_initialized = true;
        ESP_LOGI(TAG, "BMP280 센서 초기화 성공 (주소: 0x%02X)", bmp280_sensor->getAddress());
    } else {
        ESP_LOGW(TAG, "BMP280 센서 초기화 실패");
    }
    
    // 결과 요약
    uint8_t working_sensors = getWorkingSensorCount();
    if (working_sensors > 0) {
        ESP_LOGI(TAG, "센서 매니저 초기화 완료! 작동 센서: %d개", working_sensors);
        if (aht20_initialized) ESP_LOGI(TAG, "   - AHT20: 온도/습도 센서");
        if (bmp280_initialized) ESP_LOGI(TAG, "   - BMP280: 기압/온도 센서");
    } else {
        ESP_LOGE(TAG, "센서 매니저 초기화 실패 - 사용 가능한 센서가 없음");
    }
    
    return (working_sensors > 0);
}

SensorData SensorManager::readAllSensors() {
    SensorData data = {};  // 모든 필드를 0으로 초기화
    data.timestamp = esp_timer_get_time() / 1000;  // microseconds to milliseconds
    data.aht20_available = false;
    data.bmp280_available = false;
    
    // AHT20 데이터 읽기
    if (aht20_initialized && aht20_sensor) {
        float temp, hum;
        if (aht20_sensor->readData(&temp, &hum)) {
            data.temperature_aht20 = temp;
            data.humidity_aht20 = hum;
            data.aht20_available = true;
            ESP_LOGD(TAG, "AHT20 읽기 성공: 온도=%.2f°C, 습도=%.2f%%", temp, hum);
        } else {
            ESP_LOGW(TAG, "AHT20 데이터 읽기 실패");
        }
    }
    
    // BMP280 데이터 읽기 (실제 구현)
    if (bmp280_initialized && bmp280_sensor) {
        float temp, press, alt;
        if (bmp280_sensor->readData(&temp, &press, &alt)) {
            data.temperature_bmp280 = temp;
            data.pressure_bmp280 = press;
            data.altitude_bmp280 = alt;
            data.bmp280_available = true;
            ESP_LOGD(TAG, "BMP280 읽기 성공: 온도=%.2f°C, 기압=%.2fhPa, 고도=%.2fm", temp, press, alt);
        } else {
            ESP_LOGW(TAG, "BMP280 데이터 읽기 실패");
        }
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
    ESP_LOGI(TAG, "센서 진단 결과:");
    ESP_LOGI(TAG, "AHT20: %s %s", 
             data.aht20_available ? "정상" : "오프라인",
             data.aht20_available ? "(온도/습도)" : "");
    
    ESP_LOGI(TAG, "BMP280: %s %s", 
             data.bmp280_available ? "정상" : "오프라인",
             data.bmp280_available ? "(기압/온도)" : "");
    
    ESP_LOGI(TAG, "전체 작동 센서: %d개", this->getWorkingSensorCount());
    
    if (data.aht20_available) {
        ESP_LOGI(TAG, "AHT20 값: 온도=%.2f°C, 습도=%.2f%%", 
                data.temperature_aht20, data.humidity_aht20);
    }
    
    if (data.bmp280_available) {
        ESP_LOGI(TAG, "BMP280 값: 온도=%.2f°C, 기압=%.2fhPa, 고도=%.2fm", 
                data.temperature_bmp280, data.pressure_bmp280, data.altitude_bmp280);
    }
    
    // 온도 비교 (두 센서 모두 사용 가능할 때)
    if (data.aht20_available && data.bmp280_available) {
        float temp_diff = data.temperature_aht20 - data.temperature_bmp280;
        ESP_LOGI(TAG, "온도 차이: %.2f°C (AHT20 - BMP280)", temp_diff);
        
        if (abs(temp_diff) > 5.0) {
            ESP_LOGW(TAG, "온도 차이가 큽니다. 센서 위치나 환경을 확인하세요.");
        }
    }
}