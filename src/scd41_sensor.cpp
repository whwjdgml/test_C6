#include "scd41_sensor.h"
#include "sensor_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

static const char *TAG = "SCD41";

// SCD41 명령어
#define SCD41_CMD_START_PERIODIC_MEASUREMENT    0x21B1
#define SCD41_CMD_READ_MEASUREMENT              0xEC05
#define SCD41_CMD_STOP_PERIODIC_MEASUREMENT     0x3F86
#define SCD41_CMD_GET_DATA_READY_STATUS         0xE4B8
#define SCD41_CMD_REINIT                        0x3646
#define SCD41_CMD_GET_SERIAL_NUMBER             0x3682
#define SCD41_CMD_SELF_TEST                     0x3639
#define SCD41_CMD_GET_AUTOMATIC_SELF_CAL        0x2313
#define SCD41_CMD_SET_AUTOMATIC_SELF_CAL        0x2416
#define SCD41_CMD_PERFORM_FORCED_RECAL          0x362F

#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_TIMEOUT_MS          1000

SCD41Sensor::SCD41Sensor() : initialized_(false) {
}

bool SCD41Sensor::init() {
    ESP_LOGI(TAG, "SCD41 센서 초기화 시작...");

    // 1. 먼저 이전 측정 상태를 중지시켜 센서를 안정적인 IDLE 상태로 만듭니다.
    //    (처음 전원을 켤 때는 실패하는 것이 정상이므로 반환값은 무시합니다.)
    stopPeriodicMeasurement();
    vTaskDelay(pdMS_TO_TICKS(500)); // 데이터시트 권장 대기 시간 (500ms)

    // 2. 장기적인 정확도를 위해 자동 자가 보정(ASC)을 활성화합니다.
    //    이 설정은 센서의 비휘발성 메모리에 저장됩니다.
    if (!setAutomaticSelfCalibration(true)) {
        ESP_LOGW(TAG, "자동 캘리브레이션(ASC) 설정에 실패했습니다. (계속 진행)");
    }

    // 3. 주기적 측정을 시작합니다.
    if (!startPeriodicMeasurement()) {
        ESP_LOGE(TAG, "주기적 측정 시작 실패");
        return false;
    }

    // 4. 첫 데이터가 준비될 때까지 대기합니다.
    ESP_LOGI(TAG, "첫 번째 측정 완료 대기 중... (약 5초)");
    vTaskDelay(pdMS_TO_TICKS(5000));

    initialized_ = true;
    ESP_LOGI(TAG, "SCD41 초기화 완료");
    return true;
}

bool SCD41Sensor::readData(float *co2, float *temperature, float *humidity) {
    if (!initialized_) {
        ESP_LOGE(TAG, "SCD41 초기화되지 않음");
        return false;
    }
    
    // 데이터 준비 상태 확인
    bool data_ready = false;
    if (!getDataReadyStatus(&data_ready)) {
        ESP_LOGE(TAG, "데이터 준비 상태 확인 실패");
        return false;
    }
    
    if (!data_ready) {
        ESP_LOGW(TAG, "데이터가 아직 준비되지 않음");
        return false;
    }
    
    // 측정값 읽기
    uint16_t co2_raw, temp_raw, hum_raw;
    if (!readMeasurement(&co2_raw, &temp_raw, &hum_raw)) {
        ESP_LOGE(TAG, "측정값 읽기 실패");
        return false;
    }
    
    // 데이터 변환
    *co2 = (float)co2_raw;
    *temperature = -45.0f + 175.0f * (float)temp_raw / 65535.0f;
    *humidity = 100.0f * (float)hum_raw / 65535.0f;
    
    // 유효성 검사
    if (!validateData(*co2, *temperature, *humidity)) {
        ESP_LOGE(TAG, "데이터 유효성 검사 실패");
        return false;
    }
    
    ESP_LOGD(TAG, "SCD41 측정: CO2=%.0fppm, 온도=%.2f°C, 습도=%.2f%%", 
             *co2, *temperature, *humidity);
    
    return true;
}

bool SCD41Sensor::reset() {
    return sendCommand(SCD41_CMD_REINIT);
}

bool SCD41Sensor::setAutomaticSelfCalibration(bool enable) {
    uint16_t value = enable ? 0x0001 : 0x0000;
    return sendCommandWithArg(SCD41_CMD_SET_AUTOMATIC_SELF_CAL, value);
}

bool SCD41Sensor::performForcedRecalibration(uint16_t co2_reference) {
    return sendCommandWithArg(SCD41_CMD_PERFORM_FORCED_RECAL, co2_reference);
}

bool SCD41Sensor::getAutomaticSelfCalibration(bool* enabled) {
    uint8_t buffer[3];
    
    if (!sendCommand(SCD41_CMD_GET_AUTOMATIC_SELF_CAL)) {
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1)); // 짧은 대기
    
    if (!readResponse(buffer, 3)) {
        return false;
    }
    
    if (calculateCRC8(buffer, 2) != buffer[2]) {
        ESP_LOGE(TAG, "ASC 상태 읽기 CRC 오류");
        return false;
    }
    
    *enabled = ((buffer[0] << 8) | buffer[1]) == 0x0001;
    return true;
}

bool SCD41Sensor::startPeriodicMeasurement() {
    return sendCommand(SCD41_CMD_START_PERIODIC_MEASUREMENT);
}

bool SCD41Sensor::stopPeriodicMeasurement() {
    return sendCommand(SCD41_CMD_STOP_PERIODIC_MEASUREMENT);
}

bool SCD41Sensor::getDataReadyStatus(bool *data_ready) {
    uint8_t buffer[3];
    
    if (!sendCommand(SCD41_CMD_GET_DATA_READY_STATUS)) {
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1)); // 짧은 대기
    
    if (!readResponse(buffer, 3)) {
        return false;
    }
    
    // CRC 확인
    if (calculateCRC8(buffer, 2) != buffer[2]) {
        ESP_LOGE(TAG, "데이터 준비 상태 CRC 오류");
        return false;
    }
    
    uint16_t status = (buffer[0] << 8) | buffer[1];
    *data_ready = ((status & 0x07FF) != 0);
    
    return true;
}

bool SCD41Sensor::readMeasurement(uint16_t *co2, uint16_t *temp, uint16_t *hum) {
    uint8_t buffer[9];
    
    if (!sendCommand(SCD41_CMD_READ_MEASUREMENT)) {
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1)); // 짧은 대기
    
    if (!readResponse(buffer, 9)) {
        return false;
    }
    
    // CRC 검증 (각 워드마다)
    for (int i = 0; i < 3; i++) {
        if (calculateCRC8(&buffer[i * 3], 2) != buffer[i * 3 + 2]) {
            ESP_LOGE(TAG, "측정값 CRC 오류 (워드 %d)", i);
            return false;
        }
    }
    
    // 데이터 조합
    *co2 = (buffer[0] << 8) | buffer[1];
    *temp = (buffer[3] << 8) | buffer[4];
    *hum = (buffer[6] << 8) | buffer[7];
    
    return true;
}

bool SCD41Sensor::sendCommand(uint16_t command) {
    uint8_t cmd_bytes[2] = {(uint8_t)(command >> 8), (uint8_t)(command & 0xFF)};
    esp_err_t ret;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SCD41_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, cmd_bytes, 2, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return (ret == ESP_OK);
}

bool SCD41Sensor::sendCommandWithArg(uint16_t command, uint16_t arg) {
    uint8_t data[5];
    data[0] = (uint8_t)(command >> 8);
    data[1] = (uint8_t)(command & 0xFF);
    data[2] = (uint8_t)(arg >> 8);
    data[3] = (uint8_t)(arg & 0xFF);
    data[4] = calculateCRC8(&data[2], 2);
    
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SCD41_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, sizeof(data), true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return (ret == ESP_OK);
}

bool SCD41Sensor::readResponse(uint8_t *data, size_t len) {
    esp_err_t ret;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SCD41_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return (ret == ESP_OK);
}

uint8_t SCD41Sensor::calculateCRC8(const uint8_t *data, size_t len) {
    uint8_t crc = 0xFF;
    
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc = (crc << 1);
            }
        }
    }
    
    return crc;
}

bool SCD41Sensor::validateData(float co2, float temperature, float humidity) {
    return (co2 >= 400.0f && co2 <= 5000.0f &&        // CO2 범위
            temperature >= -10.0f && temperature <= 60.0f && // 온도 범위  
            humidity >= 0.0f && humidity <= 100.0f);    // 습도 범위
}