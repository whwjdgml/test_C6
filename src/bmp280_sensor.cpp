/**
 * @file bmp280_sensor.cpp
 * @brief BMP280 기압 및 온도 센서 클래스의 구현부
 *
 * BMP280Sensor 클래스의 멤버 함수들을 실제로 구현합니다.
 * I2C 통신을 통해 센서를 제어하고, 데이터를 수신하여 처리하는
 * 로직이 포함되어 있습니다.
 */
#include "bmp280_sensor.h"
#include "sensor_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include <cmath>

// 로그 태그
static const char *TAG = "BMP280";

// BMP280 레지스터 주소 정의
#define BMP280_REG_ID           0xD0 ///< 칩 ID 레지스터
#define BMP280_REG_RESET        0xE0 ///< 리셋 레지스터
#define BMP280_REG_STATUS       0xF3 ///< 상태 레지스터
#define BMP280_REG_CTRL_MEAS    0xF4 ///< 측정 제어 레지스터
#define BMP280_REG_CONFIG       0xF5 ///< 설정 레지스터
#define BMP280_REG_PRESS_MSB    0xF7 ///< 기압 데이터 MSB
#define BMP280_REG_TEMP_MSB     0xFA ///< 온도 데이터 MSB

// 캘리브레이션 데이터 시작 주소
#define BMP280_REG_CALIB_START  0x88

#define I2C_MASTER_NUM          I2C_NUM_0   ///< I2C 마스터 번호
#define I2C_TIMEOUT_MS          1000        ///< I2C 통신 타임아웃 (ms)

/**
 * @brief BMP280Sensor 클래스 생성자
 */
BMP280Sensor::BMP280Sensor() : initialized_(false), detected_addr_(0) {
}

/**
 * @brief BMP280 센서를 초기화합니다.
 *
 * 센서 감지, 리셋, 캘리브레이션 데이터 로드, 센서 설정을 순차적으로 수행합니다.
 * @return bool 초기화 성공 시 true, 실패 시 false
 */
bool BMP280Sensor::init() {
    ESP_LOGI(TAG, "BMP280 센서 초기화 시작...");
    
    // 1. I2C 주소로 센서 감지
    if (!detectSensor()) {
        ESP_LOGE(TAG, "BMP280 센서를 찾을 수 없음");
        return false;
    }
    ESP_LOGI(TAG, "BMP280 발견: 주소 0x%02X", detected_addr_);
    
    // 2. 소프트 리셋
    if (!reset()) {
        ESP_LOGE(TAG, "BMP280 리셋 실패");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // 리셋 후 안정화 대기
    
    // 3. 캘리브레이션 데이터 읽기
    if (!readCalibrationData()) {
        ESP_LOGE(TAG, "캘리브레이션 데이터 읽기 실패");
        return false;
    }
    
    // 4. 센서 측정 모드 및 필터 설정
    if (!configureSensor()) {
        ESP_LOGE(TAG, "센서 설정 실패");
        return false;
    }
    
    initialized_ = true;
    ESP_LOGI(TAG, "BMP280 초기화 완료");
    return true;
}

/**
 * @brief BMP280 센서로부터 온도, 기압, 고도 데이터를 읽습니다.
 *
 * 원시 데이터를 읽고, 보정 계산을 거쳐 실제 물리량으로 변환합니다.
 * @param temperature 변환된 온도 값을 저장할 포인터 (°C)
 * @param pressure 변환된 기압 값을 저장할 포인터 (hPa)
 * @param altitude 계산된 고도 값을 저장할 포인터 (m)
 * @return bool 데이터 읽기 및 변환 성공 시 true, 실패 시 false
 */
bool BMP280Sensor::readData(float *temperature, float *pressure, float *altitude) {
    if (!initialized_) {
        ESP_LOGE(TAG, "BMP280 초기화되지 않음");
        return false;
    }
    
    // 1. 원시 ADC 값 읽기
    int32_t adc_temp, adc_press;
    if (!readRawData(&adc_temp, &adc_press)) {
        ESP_LOGE(TAG, "원시 데이터 읽기 실패");
        return false;
    }
    
    // 2. 온도 보정
    int32_t t_fine = compensateTemperature(adc_temp);
    *temperature = (float)t_fine / 5120.0f;
    
    // 3. 기압 보정
    uint32_t press_comp = compensatePressure(adc_press, t_fine);
    *pressure = (float)press_comp / 256.0f / 100.0f; // Pa를 hPa로 변환
    
    // 4. 고도 계산
    *altitude = calculateAltitude(*pressure);
    
    // 5. 데이터 유효성 검사
    if (!validateData(*temperature, *pressure)) {
        ESP_LOGW(TAG, "데이터 유효성 검사 실패 - T:%.2f, P:%.2f", *temperature, *pressure);
    }
    
    return true;
}

/**
 * @brief BMP280 센서를 소프트 리셋합니다.
 *
 * @return bool 리셋 명령 전송 성공 시 true, 실패 시 false
 */
bool BMP280Sensor::reset() {
    uint8_t reset_cmd = 0xB6; // BMP280 리셋 명령어
    esp_err_t ret;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (detected_addr_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP280_REG_RESET, true);
    i2c_master_write_byte(cmd, reset_cmd, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return (ret == ESP_OK);
}

/**
 * @brief 가능한 I2C 주소(0x76, 0x77)를 스캔하여 BMP280 센서를 감지합니다.
 *
 * 칩 ID 레지스터를 읽어 0x58 값이 반환되는지 확인합니다.
 * @return bool 센서 감지 성공 시 true, 실패 시 false
 */
bool BMP280Sensor::detectSensor() {
    uint8_t addresses[] = {BMP280_ADDR_1, BMP280_ADDR_0}; // 데이터시트 상 0x77, 0x76
    
    for (int i = 0; i < 2; i++) {
        uint8_t addr = addresses[i];
        uint8_t chip_id;
        esp_err_t ret;
        
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, BMP280_REG_ID, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
        i2c_master_read_byte(cmd, &chip_id, I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK && chip_id == 0x58) { // BMP280의 칩 ID는 0x58
            detected_addr_ = addr;
            ESP_LOGI(TAG, "BMP280 ID 확인: 0x%02X (주소: 0x%02X)", chip_id, addr);
            return true;
        }
    }
    
    return false;
}

/**
 * @brief 센서의 비휘발성 메모리에서 24바이트의 캘리브레이션 데이터를 읽습니다.
 *
 * @return bool 데이터 읽기 및 파싱 성공 시 true, 실패 시 false
 */
bool BMP280Sensor::readCalibrationData() {
    uint8_t calib_data[24];
    esp_err_t ret;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (detected_addr_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP280_REG_CALIB_START, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (detected_addr_ << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, calib_data, 24, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        return false;
    }
    
    // 데이터시트에 따라 little-endian 형식의 캘리브레이션 데이터를 파싱
    calib_data_.dig_T1 = (calib_data[1] << 8) | calib_data[0];
    calib_data_.dig_T2 = (calib_data[3] << 8) | calib_data[2];
    calib_data_.dig_T3 = (calib_data[5] << 8) | calib_data[4];
    
    calib_data_.dig_P1 = (calib_data[7] << 8) | calib_data[6];
    calib_data_.dig_P2 = (calib_data[9] << 8) | calib_data[8];
    calib_data_.dig_P3 = (calib_data[11] << 8) | calib_data[10];
    calib_data_.dig_P4 = (calib_data[13] << 8) | calib_data[12];
    calib_data_.dig_P5 = (calib_data[15] << 8) | calib_data[14];
    calib_data_.dig_P6 = (calib_data[17] << 8) | calib_data[16];
    calib_data_.dig_P7 = (calib_data[19] << 8) | calib_data[18];
    calib_data_.dig_P8 = (calib_data[21] << 8) | calib_data[20];
    calib_data_.dig_P9 = (calib_data[23] << 8) | calib_data[22];
    
    ESP_LOGD(TAG, "캘리브레이션 데이터 로드 완료");
    return true;
}

/**
 * @brief 센서의 측정 관련 설정을 구성합니다.
 *
 * 대기 시간, IIR 필터, 오버샘플링, 측정 모드를 설정합니다.
 * @return bool 설정 성공 시 true, 실패 시 false
 */
bool BMP280Sensor::configureSensor() {
    esp_err_t ret;
    
    // 1. Config 레지스터 설정 (t_sb, filter, spi3w_en)
    // t_sb=000(0.5ms), filter=000(off)
    uint8_t config_reg = 0x00;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (detected_addr_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP280_REG_CONFIG, true);
    i2c_master_write_byte(cmd, config_reg, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) return false;
    
    // 2. Ctrl_meas 레지스터 설정 (osrs_t, osrs_p, mode)
    // osrs_t=001(x1), osrs_p=001(x1), mode=11(normal)
    uint8_t ctrl_meas_reg = 0x27;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (detected_addr_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP280_REG_CTRL_MEAS, true);
    i2c_master_write_byte(cmd, ctrl_meas_reg, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return (ret == ESP_OK);
}

/**
 * @brief 센서로부터 6바이트의 원시 측정 데이터를 읽습니다.
 *
 * @param adc_temp 원시 온도 ADC 값을 저장할 포인터
 * @param adc_press 원시 기압 ADC 값을 저장할 포인터
 * @return bool 데이터 읽기 성공 시 true, 실패 시 false
 */
bool BMP280Sensor::readRawData(int32_t *adc_temp, int32_t *adc_press) {
    uint8_t data[6];
    esp_err_t ret;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (detected_addr_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP280_REG_PRESS_MSB, true); // 시작 주소
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (detected_addr_ << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK); // 6바이트 연속 읽기
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        return false;
    }
    
    // 20비트 ADC 값을 32비트 정수형으로 조합
    *adc_press = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);
    *adc_temp = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | ((int32_t)data[5] >> 4);
    
    return true;
}

/**
 * @brief 원시 온도 ADC 값을 보정하여 t_fine 값을 계산합니다.
 *
 * 이 t_fine 값은 실제 온도 계산 및 기압 보정에 사용됩니다.
 * @param adc_temp 원시 온도 ADC 값
 * @return int32_t 보정된 온도 값 (t_fine)
 */
int32_t BMP280Sensor::compensateTemperature(int32_t adc_temp) {
    int32_t var1, var2;
    
    var1 = ((((adc_temp >> 3) - ((int32_t)calib_data_.dig_T1 << 1))) * ((int32_t)calib_data_.dig_T2)) >> 11;
    var2 = (((((adc_temp >> 4) - ((int32_t)calib_data_.dig_T1)) * ((adc_temp >> 4) - ((int32_t)calib_data_.dig_T1))) >> 12) * ((int32_t)calib_data_.dig_T3)) >> 14;
    
    return var1 + var2;
}

/**
 * @brief 원시 기압 ADC 값을 보정하여 실제 기압 값을 계산합니다.
 *
 * 데이터시트에 명시된 64비트 정수형 계산을 사용합니다.
 * @param adc_press 원시 기압 ADC 값
 * @param t_fine compensateTemperature에서 계산된 t_fine 값
 * @return uint32_t 보정된 기압 값 (단위: Pa, Q24.8 포맷)
 */
uint32_t BMP280Sensor::compensatePressure(int32_t adc_press, int32_t t_fine) {
    int64_t var1, var2, p;
    
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib_data_.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib_data_.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib_data_.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib_data_.dig_P3) >> 8) + ((var1 * (int64_t)calib_data_.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib_data_.dig_P1) >> 33;
    
    if (var1 == 0) {
        return 0; // 0으로 나누기 오류 방지
    }
    
    p = 1048576 - adc_press;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib_data_.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib_data_.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib_data_.dig_P7) << 4);
    
    return (uint32_t)p;
}

/**
 * @brief 기압 값을 이용하여 고도를 계산합니다.
 *
 * 국제 표준 대기 모델 공식을 사용합니다.
 * @param pressure_hpa 헥토파스칼(hPa) 단위의 현재 기압
 * @return float 미터(m) 단위의 계산된 고도
 */
float BMP280Sensor::calculateAltitude(float pressure_hpa) {
    const float SEA_LEVEL_PRESSURE = 1013.25f; // 해수면 평균 기압 (hPa)
    // 고도(m) = 44330 * (1 - (P/P0)^(1/5.255))
    return 44330.0f * (1.0f - powf(pressure_hpa / SEA_LEVEL_PRESSURE, 0.1903f));
}

/**
 * @brief 변환된 데이터의 유효 범위를 확인합니다.
 *
 * @param temperature 검사할 온도 값
 * @param pressure 검사할 기압 값
 * @return bool 데이터가 유효 범위 내에 있으면 true, 아니면 false
 */
bool BMP280Sensor::validateData(float temperature, float pressure) {
    // BMP280 데이터시트에 명시된 유효 측정 범위
    return (temperature >= -40.0f && temperature <= 85.0f && 
            pressure >= 300.0f && pressure <= 1100.0f);
}