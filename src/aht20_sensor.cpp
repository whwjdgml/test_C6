#include "aht20_sensor.h"
#include "sensor_config.h"
#include "i2c_wrapper.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "AHT20";

// AHT20 명령어
static constexpr uint8_t AHT20_INIT_CMD      = 0xBE;
static constexpr uint8_t AHT20_TRIGGER_CMD   = 0xAC;
static constexpr uint8_t AHT20_SOFT_RESET    = 0xBA;
static constexpr uint8_t AHT20_STATUS_CMD    = 0x71;

// AHT20 상태 비트
static constexpr uint8_t AHT20_STATUS_BUSY   = 0x80;
static constexpr uint8_t AHT20_STATUS_CAL    = 0x08;

#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_TIMEOUT_MS      1000

AHT20Sensor::AHT20Sensor() : initialized_(false) {
}

bool AHT20Sensor::init() {
    ESP_LOGI(TAG, "AHT20 센서 초기화 시작...");
    
    // 소프트 리셋
    if (!reset()) {
        ESP_LOGE(TAG, "AHT20 리셋 실패");
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(40)); // 40ms 대기
    
    // 초기화 명령 전송
    if (!sendInitCommand()) {
        ESP_LOGE(TAG, "AHT20 초기화 명령 실패");
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10)); // 10ms 대기
    
    // 상태 확인
    uint8_t status;
    esp_err_t ret;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT20_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, AHT20_STATUS_CMD, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT20_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &status, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AHT20 상태 읽기 실패: %s", esp_err_to_name(ret));
        return false;
    }
    
    if (!(status & AHT20_STATUS_CAL)) {
        ESP_LOGE(TAG, "AHT20 캘리브레이션 상태 확인 실패 (status: 0x%02X)", status);
        return false;
    }
    
    initialized_ = true;
    ESP_LOGI(TAG, "AHT20 초기화 완료 (status: 0x%02X)", status);
    return true;
}

bool AHT20Sensor::readData(float *temperature, float *humidity) {
    if (!initialized_) {
        ESP_LOGE(TAG, "AHT20 초기화되지 않음");
        return false;
    }
    
    // 측정 트리거
    if (!triggerMeasurement()) {
        ESP_LOGE(TAG, "측정 트리거 실패");
        return false;
    }
    
    // 측정 완료 대기 (최대 80ms)
    for (int i = 0; i < 8; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        
        uint8_t status;
        esp_err_t ret;
        
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (AHT20_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read_byte(cmd, &status, I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK && !(status & AHT20_STATUS_BUSY)) {
            break;
        }
        
        if (i == 7) {
            ESP_LOGE(TAG, "측정 시간 초과");
            return false;
        }
    }
    
    // 데이터 읽기
    uint8_t raw_data[6];
    if (!readRawData(raw_data)) {
        ESP_LOGE(TAG, "원시 데이터 읽기 실패");
        return false;
    }
    
    // 데이터 변환
    convertRawData(raw_data, temperature, humidity);
    
    // 유효성 검사
    if (!validateData(*temperature, *humidity)) {
        ESP_LOGE(TAG, "데이터 유효성 검사 실패");
        return false;
    }
    
    return true;
}

bool AHT20Sensor::reset() {
    esp_err_t ret;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT20_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, AHT20_SOFT_RESET, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        initialized_ = false;
        ESP_LOGI(TAG, "AHT20 소프트 리셋 완료");
    }
    
    return (ret == ESP_OK);
}

bool AHT20Sensor::sendInitCommand() {
    uint8_t init_data[] = {AHT20_INIT_CMD, 0x08, 0x00};
    esp_err_t ret;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT20_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, init_data, sizeof(init_data), true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return (ret == ESP_OK);
}

bool AHT20Sensor::triggerMeasurement() {
    uint8_t trigger_data[] = {AHT20_TRIGGER_CMD, 0x33, 0x00};
    esp_err_t ret;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT20_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, trigger_data, sizeof(trigger_data), true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return (ret == ESP_OK);
}

bool AHT20Sensor::readRawData(uint8_t *data) {
    esp_err_t ret;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT20_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return (ret == ESP_OK);
}

void AHT20Sensor::convertRawData(uint8_t *raw_data, float *temperature, float *humidity) {
    // 습도 데이터 (20비트)
    uint32_t humidity_raw = ((uint32_t)raw_data[1] << 12) | 
                           ((uint32_t)raw_data[2] << 4) | 
                           ((uint32_t)raw_data[3] >> 4);
    
    // 온도 데이터 (20비트)
    uint32_t temperature_raw = (((uint32_t)raw_data[3] & 0x0F) << 16) | 
                              ((uint32_t)raw_data[4] << 8) | 
                              ((uint32_t)raw_data[5]);
    
    // 습도 계산 (0-100%)
    *humidity = ((float)humidity_raw / 1048576.0) * 100.0;
    
    // 온도 계산 (-40 to +85°C)
    *temperature = ((float)temperature_raw / 1048576.0) * 200.0 - 50.0;
}

bool AHT20Sensor::validateData(float temperature, float humidity) {
    return (temperature >= -40.0 && temperature <= 85.0 && 
            humidity >= 0.0 && humidity <= 100.0);
}