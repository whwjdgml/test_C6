#include "sgp40_sensor.h"
#include "sensor_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

static const char *TAG = "SGP40";

// SGP40 명령어
#define SGP40_CMD_MEASURE_RAW   0x260E
#define SGP40_CMD_SELF_TEST     0x280E

#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_TIMEOUT_MS          1000

SGP40Sensor::SGP40Sensor() : initialized_(false) {
}

bool SGP40Sensor::init() {
    ESP_LOGI(TAG, "SGP40 센서 초기화 시작...");

    // 센서 자가 진단 실행
    if (!executeSelfTest()) {
        ESP_LOGE(TAG, "SGP40 자가 진단 실패");
        return false;
    }

    // VOC 알고리즘 초기화
    GasIndexAlgorithm_init(&voc_params_, GasIndexAlgorithm_ALGORITHM_TYPE_VOC);

    initialized_ = true;
    ESP_LOGI(TAG, "SGP40 초기화 완료");
    return true;
}

bool SGP40Sensor::readData(float *voc_index, float humidity, float temperature) {
    if (!initialized_) {
        ESP_LOGE(TAG, "SGP40 초기화되지 않음");
        return false;
    }

    // 온도/습도 값을 SGP40이 요구하는 틱(tick) 값으로 변환
    uint16_t rh_ticks = (uint16_t)((humidity * 65535.0f) / 100.0f + 0.5f);
    uint16_t temp_ticks = (uint16_t)(((temperature + 45.0f) * 65535.0f) / 175.0f + 0.5f);

    // 측정 명령 전송
    if (!sendCommand(SGP40_CMD_MEASURE_RAW, rh_ticks, temp_ticks)) {
        ESP_LOGE(TAG, "측정 명령 전송 실패");
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(30)); // 측정 대기 (데이터시트: 26ms)

    // 결과 읽기
    uint8_t buffer[3];
    if (!readResponse(buffer, 3)) {
        ESP_LOGE(TAG, "측정 결과 읽기 실패");
        return false;
    }

    // CRC 검증
    if (calculateCRC8(buffer, 2) != buffer[2]) {
        ESP_LOGE(TAG, "측정값 CRC 오류");
        return false;
    }

    uint16_t sraw_voc = (buffer[0] << 8) | buffer[1];

    // VOC 알고리즘 처리
    int32_t voc_index_value;
    GasIndexAlgorithm_process(&voc_params_, sraw_voc, &voc_index_value);
    *voc_index = (float)voc_index_value;

    if (!validateData(*voc_index)) {
        ESP_LOGE(TAG, "데이터 유효성 검사 실패 (VOC Index: %.0f)", *voc_index);
        return false;
    }

    ESP_LOGD(TAG, "SGP40 측정: SRAW_VOC=%u, VOC_Index=%.0f", sraw_voc, *voc_index);
    return true;
}

bool SGP40Sensor::executeSelfTest() {
    uint8_t cmd_bytes[2] = {(uint8_t)(SGP40_CMD_SELF_TEST >> 8), (uint8_t)(SGP40_CMD_SELF_TEST & 0xFF)};
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SGP40_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, cmd_bytes, 2, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) return false;

    vTaskDelay(pdMS_TO_TICKS(250)); // 자가 진단 대기

    uint8_t buffer[3];
    if (!readResponse(buffer, 3)) return false;

    if (calculateCRC8(buffer, 2) != buffer[2]) {
        ESP_LOGE(TAG, "자가 진단 응답 CRC 오류");
        return false;
    }

    uint16_t result = (buffer[0] << 8) | buffer[1];
    if (result != 0xD400) {
        ESP_LOGE(TAG, "자가 진단 실패 코드: 0x%04X", result);
        return false;
    }

    ESP_LOGI(TAG, "SGP40 자가 진단 통과");
    return true;
}

bool SGP40Sensor::sendCommand(uint16_t command, uint16_t arg1, uint16_t arg2) {
    uint8_t data[8];
    data[0] = (uint8_t)(command >> 8);
    data[1] = (uint8_t)(command & 0xFF);
    data[2] = (uint8_t)(arg1 >> 8);
    data[3] = (uint8_t)(arg1 & 0xFF);
    data[4] = calculateCRC8(&data[2], 2);
    data[5] = (uint8_t)(arg2 >> 8);
    data[6] = (uint8_t)(arg2 & 0xFF);
    data[7] = calculateCRC8(&data[5], 2);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SGP40_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, sizeof(data), true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK);
}

bool SGP40Sensor::readResponse(uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SGP40_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK);
}

uint8_t SGP40Sensor::calculateCRC8(const uint8_t *data, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x31;
            else crc = (crc << 1);
        }
    }
    return crc;
}

bool SGP40Sensor::validateData(float voc_index) {
    return (voc_index >= 0.0f && voc_index <= 500.0f);
}