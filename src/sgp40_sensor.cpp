/**
 * @file sgp40_sensor.cpp
 * @brief SGP40 VOC 센서 클래스의 구현부
 *
 * SGP40Sensor 클래스의 멤버 함수들을 실제로 구현합니다.
 * I2C 통신, 자체 테스트, 측정 및 Sensirion의 VOC 알고리즘 연동 로직이
 * 포함되어 있습니다.
 */
#include "sgp40_sensor.h"
#include "sensor_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

// 로그 태그
static const char *TAG = "SGP40";

// SGP40 I2C 명령어 정의
#define SGP40_CMD_MEASURE_RAW   0x260E ///< 원시 데이터 측정 (습도/온도 보정값 필요)
#define SGP40_CMD_SELF_TEST     0x280E ///< 자체 테스트 실행

#define I2C_MASTER_NUM          I2C_NUM_0   ///< I2C 마스터 번호
#define I2C_TIMEOUT_MS          1000        ///< I2C 통신 타임아웃 (ms)

/**
 * @brief SGP40Sensor 클래스 생성자
 */
SGP40Sensor::SGP40Sensor() : initialized_(false) {
    // VOC 알고리즘의 상태를 기본값으로 초기화합니다.
    GasIndexAlgorithm_init(&voc_params_, GasIndexAlgorithm_ALGORITHM_TYPE_VOC);
}

/**
 * @brief SGP40 센서를 초기화합니다.
 *
 * 자체 테스트를 실행하여 센서의 정상 작동을 확인하고,
 * VOC 알고리즘을 초기 상태로 설정합니다.
 * @return bool 초기화 성공 시 true, 실패 시 false
 */
bool SGP40Sensor::init() {
    ESP_LOGI(TAG, "SGP40 센서 초기화 시작...");

    // 1. 센서 자체 테스트 실행
    if (!executeSelfTest()) {
        ESP_LOGE(TAG, "SGP40 자체 테스트 실패");
        return false;
    }

    // 2. VOC 알고리즘 파라미터 초기화
    // 생성자에서 이미 호출되었지만, 재초기화가 필요할 경우를 대비해
    // 여기에 로직을 두는 것을 고려할 수 있습니다.
    // GasIndexAlgorithm_init(&voc_params_, GasIndexAlgorithm_ALGORITHM_TYPE_VOC);

    initialized_ = true;
    ESP_LOGI(TAG, "SGP40 초기화 완료");
    return true;
}

/**
 * @brief SGP40 센서로부터 VOC 인덱스를 읽습니다.
 *
 * 외부에서 제공된 온도와 습도 값으로 측정 환경을 보정한 후,
 * 원시 VOC 값을 읽어 알고리즘을 통해 VOC 인덱스로 변환합니다.
 * @param voc_index 계산된 VOC 인덱스를 저장할 포인터
 * @param humidity 현재 상대 습도 (%RH)
 * @param temperature 현재 온도 (°C)
 * @return bool 데이터 읽기 및 계산 성공 시 true, 실패 시 false
 */
bool SGP40Sensor::readData(float *voc_index, float humidity, float temperature) {
    if (!initialized_) {
        ESP_LOGE(TAG, "SGP40 초기화되지 않음");
        return false;
    }

    // 1. 온도/습도 값을 SGP40이 요구하는 16비트 틱(tick) 값으로 변환
    // 습도: ticks = %RH * 65535 / 100
    uint16_t rh_ticks = (uint16_t)((humidity * 65535.0f) / 100.0f + 0.5f);
    // 온도: ticks = (T + 45) * 65535 / 175
    uint16_t temp_ticks = (uint16_t)(((temperature + 45.0f) * 65535.0f) / 175.0f + 0.5f);

    // 2. 보정된 측정 명령 전송
    if (!sendCommand(SGP40_CMD_MEASURE_RAW, rh_ticks, temp_ticks)) {
        ESP_LOGE(TAG, "측정 명령 전송 실패");
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(30)); // 데이터시트 권장 측정 시간 (최대 30ms)

    // 3. 측정 결과(원시 데이터) 읽기
    uint8_t buffer[3];
    if (!readResponse(buffer, 3)) {
        ESP_LOGE(TAG, "측정 결과 읽기 실패");
        return false;
    }

    // 4. CRC 검증
    if (calculateCRC8(buffer, 2) != buffer[2]) {
        ESP_LOGE(TAG, "측정값 CRC 오류");
        return false;
    }

    uint16_t sraw_voc = (buffer[0] << 8) | buffer[1];

    // 5. VOC 알고리즘을 통해 원시 값을 VOC 인덱스로 변환
    int32_t voc_index_value;
    GasIndexAlgorithm_process(&voc_params_, sraw_voc, &voc_index_value);
    *voc_index = (float)voc_index_value;

    // 6. 데이터 유효성 검사
    if (!validateData(*voc_index)) {
        ESP_LOGW(TAG, "데이터 유효성 검사 실패 (VOC Index: %.0f)", *voc_index);
    }

    ESP_LOGD(TAG, "SGP40 측정: SRAW_VOC=%u, VOC_Index=%.0f", sraw_voc, *voc_index);
    return true;
}

/**
 * @brief 센서 자체 테스트를 실행하고 결과를 확인합니다.
 * @return bool 테스트 통과 시 true, 실패 시 false
 */
bool SGP40Sensor::executeSelfTest() {
    uint8_t cmd_bytes[2] = {(uint8_t)(SGP40_CMD_SELF_TEST >> 8), (uint8_t)(SGP40_CMD_SELF_TEST & 0xFF)};
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SGP40_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, cmd_bytes, 2, true);
    i2c_master_stop(cmd);
    if (i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != ESP_OK) {
        i2c_cmd_link_delete(cmd);
        return false;
    }
    i2c_cmd_link_delete(cmd);

    vTaskDelay(pdMS_TO_TICKS(250)); // 데이터시트 권장 자가 진단 시간 (최대 250ms)

    uint8_t buffer[3];
    if (!readResponse(buffer, 3)) return false;

    if (calculateCRC8(buffer, 2) != buffer[2]) {
        ESP_LOGE(TAG, "자가 진단 응답 CRC 오류");
        return false;
    }

    uint16_t result = (buffer[0] << 8) | buffer[1];
    if (result != 0xD400) { // 성공 시 반환값은 0xD400
        ESP_LOGE(TAG, "자가 진단 실패 코드: 0x%04X", result);
        return false;
    }

    ESP_LOGI(TAG, "SGP40 자가 진단 통과");
    return true;
}

/**
 * @brief SGP40에 측정 명령과 보정 데이터를 전송합니다.
 * @param command 16비트 명령어 (SGP40_CMD_MEASURE_RAW)
 * @param arg1 16비트 상대습도 틱 값
 * @param arg2 16비트 온도 틱 값
 * @return bool I2C 전송 성공 시 true
 */
bool SGP40Sensor::sendCommand(uint16_t command, uint16_t arg1, uint16_t arg2) {
    uint8_t data[8];
    data[0] = (uint8_t)(command >> 8);
    data[1] = (uint8_t)(command & 0xFF);
    data[2] = (uint8_t)(arg1 >> 8);
    data[3] = (uint8_t)(arg1 & 0xFF);
    data[4] = calculateCRC8(&data[2], 2); // 습도 CRC
    data[5] = (uint8_t)(arg2 >> 8);
    data[6] = (uint8_t)(arg2 & 0xFF);
    data[7] = calculateCRC8(&data[5], 2); // 온도 CRC

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SGP40_ADDR << 1) | I2C_MASTER_WRITE, true);
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

/**
 * @brief Sensirion CRC-8 (polynomial 0x31, init 0xFF) 체크섬을 계산합니다.
 * @param data CRC를 계산할 데이터
 * @param len 데이터 길이
 * @return uint8_t 계산된 CRC 값
 */
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

/**
 * @brief 계산된 VOC 인덱스의 유효 범위를 확인합니다.
 * @param voc_index 검사할 VOC 인덱스 값
 * @return bool 데이터가 유효 범위(0-500) 내에 있으면 true
 */
bool SGP40Sensor::validateData(float voc_index) {
    return (voc_index >= 0.0f && voc_index <= 500.0f);
}