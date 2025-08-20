#include "bmp280_sensor.h"
#include "sensor_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include <cmath>

static const char *TAG = "BMP280";

// BMP280 레지스터 주소
#define BMP280_REG_ID           0xD0
#define BMP280_REG_RESET        0xE0
#define BMP280_REG_STATUS       0xF3
#define BMP280_REG_CTRL_MEAS    0xF4
#define BMP280_REG_CONFIG       0xF5
#define BMP280_REG_PRESS_MSB    0xF7
#define BMP280_REG_PRESS_LSB    0xF8
#define BMP280_REG_PRESS_XLSB   0xF9
#define BMP280_REG_TEMP_MSB     0xFA
#define BMP280_REG_TEMP_LSB     0xFB
#define BMP280_REG_TEMP_XLSB    0xFC

// 캘리브레이션 데이터 시작 주소
#define BMP280_REG_CALIB_START  0x88

#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_TIMEOUT_MS          1000

BMP280Sensor::BMP280Sensor() : initialized_(false), detected_addr_(0) {
}

bool BMP280Sensor::init() {
    ESP_LOGI(TAG, "BMP280 센서 초기화 시작...");
    
    // 센서 탐지
    if (!detectSensor()) {
        ESP_LOGE(TAG, "BMP280 센서를 찾을 수 없음");
        return false;
    }
    
    ESP_LOGI(TAG, "BMP280 발견: 주소 0x%02X", detected_addr_);
    
    // 소프트 리셋
    if (!reset()) {
        ESP_LOGE(TAG, "BMP280 리셋 실패");
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10)); // 리셋 후 대기
    
    // 캘리브레이션 데이터 읽기
    if (!readCalibrationData()) {
        ESP_LOGE(TAG, "캘리브레이션 데이터 읽기 실패");
        return false;
    }
    
    // 센서 설정
    if (!configureSensor()) {
        ESP_LOGE(TAG, "센서 설정 실패");
        return false;
    }
    
    initialized_ = true;
    ESP_LOGI(TAG, "BMP280 초기화 완료");
    return true;
}

bool BMP280Sensor::readData(float *temperature, float *pressure, float *altitude) {
    if (!initialized_) {
        ESP_LOGE(TAG, "BMP280 초기화되지 않음");
        return false;
    }
    
    // 원시 데이터 읽기
    int32_t adc_temp, adc_press;
    if (!readRawData(&adc_temp, &adc_press)) {
        ESP_LOGE(TAG, "원시 데이터 읽기 실패");
        return false;
    }
    
    // 온도 보정
    int32_t t_fine = compensateTemperature(adc_temp);
    *temperature = (float)t_fine / 5120.0f;
    
    // 기압 보정
    uint32_t press_comp = compensatePressure(adc_press, t_fine);
    *pressure = (float)press_comp / 256.0f / 100.0f; // Pa to hPa
    
    // 고도 계산
    *altitude = calculateAltitude(*pressure);
    
    // 유효성 검사
    if (!validateData(*temperature, *pressure)) {
        ESP_LOGE(TAG, "데이터 유효성 검사 실패");
        return false;
    }
    
    return true;
}

bool BMP280Sensor::reset() {
    uint8_t reset_cmd = 0xB6;
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

bool BMP280Sensor::detectSensor() {
    uint8_t addresses[] = {BMP280_ADDR_1, BMP280_ADDR_0}; // 0x77 먼저 시도
    
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
        
        if (ret == ESP_OK && chip_id == 0x58) {
            detected_addr_ = addr;
            ESP_LOGI(TAG, "BMP280 ID 확인: 0x%02X (주소: 0x%02X)", chip_id, addr);
            return true;
        }
    }
    
    return false;
}

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
    
    // 캘리브레이션 데이터 파싱
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
    
    ESP_LOGI(TAG, "캘리브레이션 데이터 로드 완료");
    return true;
}

bool BMP280Sensor::configureSensor() {
    esp_err_t ret;
    
    // Config 레지스터 설정 (standby time, filter)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (detected_addr_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP280_REG_CONFIG, true);
    i2c_master_write_byte(cmd, 0x00, true); // standby 0.5ms, filter off
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) return false;
    
    // Ctrl_meas 레지스터 설정 (oversampling, mode)
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (detected_addr_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP280_REG_CTRL_MEAS, true);
    i2c_master_write_byte(cmd, 0x27, true); // temp x1, press x1, normal mode
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return (ret == ESP_OK);
}

bool BMP280Sensor::readRawData(int32_t *adc_temp, int32_t *adc_press) {
    uint8_t data[6];
    esp_err_t ret;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (detected_addr_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP280_REG_PRESS_MSB, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (detected_addr_ << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        return false;
    }
    
    // 20비트 ADC 값 조합
    *adc_press = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);
    *adc_temp = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | ((int32_t)data[5] >> 4);
    
    return true;
}

int32_t BMP280Sensor::compensateTemperature(int32_t adc_temp) {
    int32_t var1, var2, t_fine;
    
    var1 = ((((adc_temp >> 3) - ((int32_t)calib_data_.dig_T1 << 1))) * ((int32_t)calib_data_.dig_T2)) >> 11;
    var2 = (((((adc_temp >> 4) - ((int32_t)calib_data_.dig_T1)) * ((adc_temp >> 4) - ((int32_t)calib_data_.dig_T1))) >> 12) * ((int32_t)calib_data_.dig_T3)) >> 14;
    t_fine = var1 + var2;
    
    return t_fine;
}

uint32_t BMP280Sensor::compensatePressure(int32_t adc_press, int32_t t_fine) {
    int64_t var1, var2, p;
    
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib_data_.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib_data_.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib_data_.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib_data_.dig_P3) >> 8) + ((var1 * (int64_t)calib_data_.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib_data_.dig_P1) >> 33;
    
    if (var1 == 0) {
        return 0; // 0으로 나누기 방지
    }
    
    p = 1048576 - adc_press;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib_data_.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib_data_.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib_data_.dig_P7) << 4);
    
    return (uint32_t)p;
}

float BMP280Sensor::calculateAltitude(float pressure_hpa) {
    const float SEA_LEVEL_PRESSURE = 1013.25f; // hPa
    return 44330.0f * (1.0f - powf(pressure_hpa / SEA_LEVEL_PRESSURE, 0.1903f));
}

bool BMP280Sensor::validateData(float temperature, float pressure) {
    return (temperature >= -40.0f && temperature <= 85.0f && 
            pressure >= 300.0f && pressure <= 1100.0f);
}