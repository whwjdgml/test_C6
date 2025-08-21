/**
 * @file scd41_sensor.cpp
 * @brief SCD41 CO2, 온도, 습도 센서 클래스의 구현부
 *
 * SCD41Sensor 클래스의 멤버 함수들을 실제로 구현합니다.
 * I2C 통신을 통해 센서를 제어하고, 데이터를 수신하여 처리하는
 * 로직이 포함되어 있습니다. Sensirion의 I2C 프로토콜(CRC 포함)을 따릅니다.
 */
#include "scd41_sensor.h"
#include "sensor_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

// 로그 태그
static const char *TAG = "SCD41";

// SCD41 I2C 명령어 정의
#define SCD41_CMD_START_PERIODIC_MEASUREMENT    0x21B1 ///< 주기적 측정 시작
#define SCD41_CMD_READ_MEASUREMENT              0xEC05 ///< 측정값 읽기
#define SCD41_CMD_STOP_PERIODIC_MEASUREMENT     0x3F86 ///< 주기적 측정 중지
#define SCD41_CMD_GET_DATA_READY_STATUS         0xE4B8 ///< 데이터 준비 상태 확인
#define SCD41_CMD_REINIT                        0x3646 ///< 센서 재초기화 (소프트 리셋)
#define SCD41_CMD_GET_SERIAL_NUMBER             0x3682 ///< 시리얼 번호 읽기
#define SCD41_CMD_SELF_TEST                     0x3639 ///< 자체 테스트 수행
#define SCD41_CMD_GET_AUTOMATIC_SELF_CAL        0x2313 ///< 자동 캘리브레이션 상태 읽기
#define SCD41_CMD_SET_AUTOMATIC_SELF_CAL        0x2416 ///< 자동 캘리브레이션 설정
#define SCD41_CMD_PERFORM_FORCED_RECAL          0x362F ///< 강제 재캘리브레이션 수행

#define I2C_MASTER_NUM          I2C_NUM_0   ///< I2C 마스터 번호
#define I2C_TIMEOUT_MS          1000        ///< I2C 통신 타임아웃 (ms)

/**
 * @brief SCD41Sensor 클래스 생성자
 */
SCD41Sensor::SCD41Sensor() : initialized_(false) {
}

/**
 * @brief SCD41 센서를 초기화합니다.
 *
 * 센서를 안정적인 상태로 만든 후, 자동 캘리브레이션을 활성화하고 주기적 측정을 시작합니다.
 * @return bool 초기화 성공 시 true, 실패 시 false
 */
bool SCD41Sensor::init() {
    ESP_LOGI(TAG, "SCD41 센서 초기화 시작...");

    // 1. 센서를 안정적인 IDLE 상태로 만들기 위해 측정을 중지합니다.
    //    (최초 전원 인가 시에는 실패할 수 있으며, 이는 정상입니다.)
    stopPeriodicMeasurement();
    vTaskDelay(pdMS_TO_TICKS(500)); // 데이터시트 권장 대기 시간 (500ms)

    // 2. 자동 자체 캘리브레이션(ASC) 활성화.
    //    이 설정은 센서의 비휘발성 메모리에 저장되어 유지됩니다.
    if (!setAutomaticSelfCalibration(true)) {
        ESP_LOGW(TAG, "자동 캘리브레이션(ASC) 설정 실패 (계속 진행)");
    }

    // 3. 주기적 측정을 시작합니다. (측정 간격 약 5초)
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

/**
 * @brief SCD41 센서로부터 CO2, 온도, 습도 데이터를 읽습니다.
 *
 * 데이터 준비 상태를 확인하고, 준비되었을 때만 측정값을 읽어 변환합니다.
 * @param co2 변환된 CO2 농도를 저장할 포인터 (ppm)
 * @param temperature 변환된 온도를 저장할 포인터 (°C)
 * @param humidity 변환된 습도를 저장할 포인터 (%)
 * @return bool 데이터 읽기 성공 시 true, 실패 시 false
 */
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
        ESP_LOGD(TAG, "데이터가 아직 준비되지 않음");
        return false; // 아직 데이터가 준비되지 않은 것은 오류가 아님
    }
    
    // 원시 측정값 읽기
    uint16_t co2_raw, temp_raw, hum_raw;
    if (!readMeasurement(&co2_raw, &temp_raw, &hum_raw)) {
        ESP_LOGE(TAG, "측정값 읽기 실패");
        return false;
    }
    
    // 데이터시트 공식에 따라 실제 값으로 변환
    *co2 = (float)co2_raw;
    *temperature = -45.0f + 175.0f * (float)temp_raw / 65535.0f;
    *humidity = 100.0f * (float)hum_raw / 65535.0f;
    
    // 데이터 유효성 검사
    if (!validateData(*co2, *temperature, *humidity)) {
        ESP_LOGW(TAG, "데이터 유효성 검사 실패 - CO2:%.0f, T:%.2f, H:%.2f", *co2, *temperature, *humidity);
    }
    
    ESP_LOGD(TAG, "SCD41 측정: CO2=%.0fppm, 온도=%.2f°C, 습도=%.2f%%", *co2, *temperature, *humidity);
    
    return true;
}

/**
 * @brief 센서를 소프트 리셋합니다.
 * @return bool 명령 전송 성공 시 true
 */
bool SCD41Sensor::reset() {
    return sendCommand(SCD41_CMD_REINIT);
}

/**
 * @brief 자동 자체 캘리브레이션(ASC)을 활성화/비활성화합니다.
 * @param enable true: 활성화, false: 비활성화
 * @return bool 명령 전송 성공 시 true
 */
bool SCD41Sensor::setAutomaticSelfCalibration(bool enable) {
    uint16_t value = enable ? 0x0001 : 0x0000;
    return sendCommandWithArg(SCD41_CMD_SET_AUTOMATIC_SELF_CAL, value);
}

/**
 * @brief 강제 재캘리브레이션(FRC)을 수행합니다.
 * @param co2_reference 기준 CO2 농도 (ppm)
 * @return bool 명령 전송 성공 시 true
 */
bool SCD41Sensor::performForcedRecalibration(uint16_t co2_reference) {
    return sendCommandWithArg(SCD41_CMD_PERFORM_FORCED_RECAL, co2_reference);
}

/**
 * @brief 자동 자체 캘리브레이션(ASC) 활성화 상태를 읽습니다.
 * @param enabled 상태를 저장할 포인터
 * @return bool 상태 읽기 및 CRC 검증 성공 시 true
 */
bool SCD41Sensor::getAutomaticSelfCalibration(bool* enabled) {
    uint8_t buffer[3];
    
    if (!sendCommand(SCD41_CMD_GET_AUTOMATIC_SELF_CAL)) return false;
    vTaskDelay(pdMS_TO_TICKS(1));
    if (!readResponse(buffer, 3)) return false;
    
    if (calculateCRC8(buffer, 2) != buffer[2]) {
        ESP_LOGE(TAG, "ASC 상태 읽기 CRC 오류");
        return false;
    }
    
    *enabled = ((buffer[0] << 8) | buffer[1]) == 0x0001;
    return true;
}

/**
 * @brief 주기적 측정을 시작합니다.
 * @return bool 명령 전송 성공 시 true
 */
bool SCD41Sensor::startPeriodicMeasurement() {
    return sendCommand(SCD41_CMD_START_PERIODIC_MEASUREMENT);
}

/**
 * @brief 주기적 측정을 중지합니다.
 * @return bool 명령 전송 성공 시 true
 */
bool SCD41Sensor::stopPeriodicMeasurement() {
    return sendCommand(SCD41_CMD_STOP_PERIODIC_MEASUREMENT);
}

/**
 * @brief 새로운 측정 데이터가 준비되었는지 확인합니다.
 * @param data_ready 데이터 준비 상태를 저장할 포인터
 * @return bool 상태 읽기 및 CRC 검증 성공 시 true
 */
bool SCD41Sensor::getDataReadyStatus(bool *data_ready) {
    uint8_t buffer[3];
    
    if (!sendCommand(SCD41_CMD_GET_DATA_READY_STATUS)) return false;
    vTaskDelay(pdMS_TO_TICKS(1));
    if (!readResponse(buffer, 3)) return false;
    
    if (calculateCRC8(buffer, 2) != buffer[2]) {
        ESP_LOGE(TAG, "데이터 준비 상태 CRC 오류");
        return false;
    }
    
    uint16_t status = (buffer[0] << 8) | buffer[1];
    *data_ready = ((status & 0x07FF) != 0);
    
    return true;
}

/**
 * @brief 센서로부터 9바이트의 측정 데이터를 읽습니다 (CO2, T, H 각각 3바이트).
 * @param co2 원시 CO2 값을 저장할 포인터
 * @param temp 원시 온도 값을 저장할 포인터
 * @param hum 원시 습도 값을 저장할 포인터
 * @return bool 데이터 읽기 및 모든 CRC 검증 성공 시 true
 */
bool SCD41Sensor::readMeasurement(uint16_t *co2, uint16_t *temp, uint16_t *hum) {
    uint8_t buffer[9];
    
    if (!sendCommand(SCD41_CMD_READ_MEASUREMENT)) return false;
    vTaskDelay(pdMS_TO_TICKS(1));
    if (!readResponse(buffer, 9)) return false;
    
    // 각 2바이트 데이터 뒤에 오는 1바이트 CRC를 검증
    for (int i = 0; i < 3; i++) {
        if (calculateCRC8(&buffer[i * 3], 2) != buffer[i * 3 + 2]) {
            ESP_LOGE(TAG, "측정값 CRC 오류 (워드 %d)", i);
            return false;
        }
    }
    
    *co2 = (buffer[0] << 8) | buffer[1];
    *temp = (buffer[3] << 8) | buffer[4];
    *hum = (buffer[6] << 8) | buffer[7];
    
    return true;
}

/**
 * @brief 센서에 2바이트 명령어를 전송합니다.
 * @param command 전송할 명령어
 * @return bool I2C 전송 성공 시 true
 */
bool SCD41Sensor::sendCommand(uint16_t command) {
    uint8_t cmd_bytes[2] = {(uint8_t)(command >> 8), (uint8_t)(command & 0xFF)};
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SCD41_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, cmd_bytes, 2, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return (ret == ESP_OK);
}

/**
 * @brief 센서에 2바이트 명령어와 2바이트 인자, 1바이트 CRC를 전송합니다.
 * @param command 전송할 명령어
 * @param arg 전송할 인자
 * @return bool I2C 전송 성공 시 true
 */
bool SCD41Sensor::sendCommandWithArg(uint16_t command, uint16_t arg) {
    uint8_t data[5];
    data[0] = (uint8_t)(command >> 8);
    data[1] = (uint8_t)(command & 0xFF);
    data[2] = (uint8_t)(arg >> 8);
    data[3] = (uint8_t)(arg & 0xFF);
    data[4] = calculateCRC8(&data[2], 2); // 인자에 대한 CRC 계산
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SCD41_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, sizeof(data), true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return (ret == ESP_OK);
}

/**
 * @brief 센서로부터 응답 데이터를 읽습니다.
 * @param data 데이터를 저장할 버퍼
 * @param len 읽을 데이터 길이
 * @return bool I2C 읽기 성공 시 true
 */
bool SCD41Sensor::readResponse(uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SCD41_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return (ret == ESP_OK);
}

/**
 * @brief Sensirion CRC-8 (polynomial 0x31, init 0xFF) 체크섬을 계산합니다.
 * @param data CRC를 계산할 데이터
 * @param len 데이터 길이
 * @return uint8_t 계산된 CRC 값
 */
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

/**
 * @brief 변환된 데이터의 유효 범위를 확인합니다.
 * @param co2 검사할 CO2 값
 * @param temperature 검사할 온도 값
 * @param humidity 검사할 습도 값
 * @return bool 데이터가 유효 범위 내에 있으면 true, 아니면 false
 */
bool SCD41Sensor::validateData(float co2, float temperature, float humidity) {
    // SCD41 데이터시트에 명시된 유효 측정 범위
    return (co2 >= 0.0f && co2 <= 40000.0f &&        // CO2 범위
            temperature >= -10.0f && temperature <= 60.0f && // 온도 범위  
            humidity >= 0.0f && humidity <= 100.0f);    // 습도 범위
}