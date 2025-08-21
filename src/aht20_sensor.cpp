/**
 * @file aht20_sensor.cpp
 * @brief AHT20 온도 및 습도 센서 클래스의 구현부
 *
 * AHT20Sensor 클래스의 멤버 함수들을 실제로 구현합니다.
 * I2C 통신을 통해 센서를 제어하고, 데이터를 수신하여 처리하는
 * 로직이 포함되어 있습니다.
 */
#include "aht20_sensor.h"
#include "sensor_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

// 로그 태그
static const char *TAG = "AHT20";

// AHT20 I2C 명령어 정의
#define AHT20_INIT_CMD      0xBE ///< 초기화 명령어
#define AHT20_TRIGGER_CMD   0xAC ///< 측정 트리거 명령어
#define AHT20_SOFT_RESET    0xBA ///< 소프트 리셋 명령어
#define AHT20_STATUS_CMD    0x71 ///< 상태 읽기 명령어

// AHT20 상태 비트 정의
#define AHT20_STATUS_BUSY   0x80 ///< 센서가 측정 중임을 나타내는 비트
#define AHT20_STATUS_CAL    0x08 ///< 센서가 캘리브레이션되었음을 나타내는 비트

#define I2C_MASTER_NUM      I2C_NUM_0   ///< I2C 마스터 번호
#define I2C_TIMEOUT_MS      1000        ///< I2C 통신 타임아웃 (ms)

/**
 * @brief AHT20Sensor 클래스 생성자
 */
AHT20Sensor::AHT20Sensor() : initialized_(false) {
}

/**
 * @brief AHT20 센서를 초기화합니다.
 *
 * 소프트 리셋 후, 초기화 명령을 보내고 캘리브레이션 상태를 확인합니다.
 * @return bool 초기화 성공 시 true, 실패 시 false
 */
bool AHT20Sensor::init() {
    ESP_LOGI(TAG, "AHT20 센서 초기화 시작...");
    
    // 1. 소프트 리셋
    if (!reset()) {
        ESP_LOGE(TAG, "AHT20 리셋 실패");
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(40)); // 데이터시트에 따른 리셋 후 대기 시간
    
    // 2. 초기화 명령 전송
    if (!sendInitCommand()) {
        ESP_LOGE(TAG, "AHT20 초기화 명령 실패");
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10)); // 명령 전송 후 대기
    
    // 3. 캘리브레이션 상태 확인
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
    
    // 상태 바이트의 캘리브레이션 비트(bit 3) 확인
    if (!(status & AHT20_STATUS_CAL)) {
        ESP_LOGE(TAG, "AHT20 캘리브레이션 상태 확인 실패 (status: 0x%02X)", status);
        return false;
    }
    
    initialized_ = true;
    ESP_LOGI(TAG, "AHT20 초기화 완료 (status: 0x%02X)", status);
    return true;
}

/**
 * @brief AHT20 센서로부터 온도와 습도 데이터를 읽습니다.
 *
 * 측정 트리거 후, 센서가 측정을 완료할 때까지 대기하고,
 * 원시 데이터를 읽어와 실제 값으로 변환합니다.
 * @param temperature 변환된 온도 값을 저장할 포인터
 * @param humidity 변환된 습도 값을 저장할 포인터
 * @return bool 데이터 읽기 및 변환 성공 시 true, 실패 시 false
 */
bool AHT20Sensor::readData(float *temperature, float *humidity) {
    if (!initialized_) {
        ESP_LOGE(TAG, "AHT20 초기화되지 않음");
        return false;
    }
    
    // 1. 측정 트리거
    if (!triggerMeasurement()) {
        ESP_LOGE(TAG, "측정 트리거 실패");
        return false;
    }
    
    // 2. 측정 완료 대기 (데이터시트 상 80ms)
    // Busy 비트(bit 7)가 0이 될 때까지 폴링
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
            break; // 측정이 완료됨
        }
        
        if (i == 7) {
            ESP_LOGE(TAG, "측정 시간 초과");
            return false; // 타임아웃
        }
    }
    
    // 3. 6바이트 원시 데이터 읽기
    uint8_t raw_data[6];
    if (!readRawData(raw_data)) {
        ESP_LOGE(TAG, "원시 데이터 읽기 실패");
        return false;
    }
    
    // 4. 원시 데이터를 실제 값으로 변환
    convertRawData(raw_data, temperature, humidity);
    
    // 5. 변환된 데이터의 유효성 검사
    if (!validateData(*temperature, *humidity)) {
        ESP_LOGW(TAG, "데이터 유효성 검사 실패 - T:%.2f, H:%.2f", *temperature, *humidity);
        // 유효하지 않은 데이터라도 일단 반환은 하되, 경고 로그를 남김
    }
    
    return true;
}

/**
 * @brief AHT20 센서를 소프트 리셋합니다.
 *
 * @return bool 리셋 명령 전송 성공 시 true, 실패 시 false
 */
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
        initialized_ = false; // 리셋 후 초기화 상태 해제
        ESP_LOGI(TAG, "AHT20 소프트 리셋 완료");
    }
    
    return (ret == ESP_OK);
}

/**
 * @brief AHT20 센서에 초기화 명령을 전송합니다.
 *
 * @return bool 명령 전송 성공 시 true, 실패 시 false
 */
bool AHT20Sensor::sendInitCommand() {
    // 0xBE, 0x08, 0x00: 초기화 명령어 및 파라미터
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

/**
 * @brief AHT20 센서에 측정 트리거 명령을 전송합니다.
 *
 * @return bool 명령 전송 성공 시 true, 실패 시 false
 */
bool AHT20Sensor::triggerMeasurement() {
    // 0xAC, 0x33, 0x00: 측정 트리거 명령어 및 파라미터
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

/**
 * @brief 센서로부터 6바이트의 원시 데이터를 읽습니다.
 *
 * @param data 원시 데이터를 저장할 6바이트 크기의 버퍼
 * @return bool 데이터 읽기 성공 시 true, 실패 시 false
 */
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

/**
 * @brief 원시 데이터를 실제 온도 및 습도 값으로 변환합니다.
 *
 * 데이터시트에 명시된 공식을 사용하여 20비트 원시 데이터를 계산합니다.
 * @param raw_data 변환할 6바이트 원시 데이터 버퍼
 * @param temperature 변환된 온도 값을 저장할 포인터
 * @param humidity 변환된 습도 값을 저장할 포인터
 */
void AHT20Sensor::convertRawData(uint8_t *raw_data, float *temperature, float *humidity) {
    // 습도 데이터 (20비트)
    // S_rh = (raw_data[1] << 12) | (raw_data[2] << 4) | (raw_data[3] >> 4)
    uint32_t humidity_raw = ((uint32_t)raw_data[1] << 12) | 
                           ((uint32_t)raw_data[2] << 4) | 
                           ((uint32_t)raw_data[3] >> 4);
    
    // 온도 데이터 (20비트)
    // S_t = ((raw_data[3] & 0x0F) << 16) | (raw_data[4] << 8) | raw_data[5]
    uint32_t temperature_raw = (((uint32_t)raw_data[3] & 0x0F) << 16) | 
                              ((uint32_t)raw_data[4] << 8) | 
                              ((uint32_t)raw_data[5]);
    
    // 습도 계산: RH = (S_rh / 2^20) * 100%
    *humidity = ((float)humidity_raw / 1048576.0) * 100.0;
    
    // 온도 계산: T = (S_t / 2^20) * 200 - 50
    *temperature = ((float)temperature_raw / 1048576.0) * 200.0 - 50.0;
}

/**
 * @brief 변환된 데이터의 유효 범위를 확인합니다.
 *
 * @param temperature 검사할 온도 값
 * @param humidity 검사할 습도 값
 * @return bool 데이터가 유효 범위 내에 있으면 true, 아니면 false
 */
bool AHT20Sensor::validateData(float temperature, float humidity) {
    // AHT20 데이터시트에 명시된 유효 측정 범위
    return (temperature >= -40.0 && temperature <= 85.0 && 
            humidity >= 0.0 && humidity <= 100.0);
}