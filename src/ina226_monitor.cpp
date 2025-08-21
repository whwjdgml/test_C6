#include "ina226_monitor.h"
#include "i2c_wrapper.h"
#include "sensor_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "INA226";

// INA226 I2C 주소 (점퍼 설정에 따라 다름)
#define INA226_ADDR_0 0x40
#define INA226_ADDR_1 0x41
#define INA226_ADDR_2 0x44
#define INA226_ADDR_3 0x45

// INA226 레지스터
#define INA226_REG_CONFIG       0x00
#define INA226_REG_SHUNT_VOLT   0x01
#define INA226_REG_BUS_VOLT     0x02
#define INA226_REG_POWER        0x03
#define INA226_REG_CURRENT      0x04
#define INA226_REG_CALIB        0x05
#define INA226_REG_MASK_EN      0x06
#define INA226_REG_ALERT_LIM    0x07
#define INA226_REG_MANUF_ID     0xFE
#define INA226_REG_DIE_ID       0xFF

#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_TIMEOUT_MS      1000

INA226_Monitor::INA226_Monitor(float shunt_resistance_ohm, float max_expected_current_a)
    : initialized_(false),
      detected_addr_(0),
      shunt_resistance_(shunt_resistance_ohm),
      max_current_(max_expected_current_a),
      current_lsb_(0.0f),
      cal_value_(0) {}

bool INA226_Monitor::init() {
    ESP_LOGI(TAG, "INA226 모니터 초기화 시작...");

    if (!detectSensor()) {
        ESP_LOGE(TAG, "INA226 센서를 찾을 수 없음");
        return false;
    }
    ESP_LOGI(TAG, "INA226 발견: 주소 0x%02X", detected_addr_);

    if (!calibrate()) {
        ESP_LOGE(TAG, "캘리브레이션 값 계산 또는 설정 실패");
        return false;
    }

    if (!configure()) {
        ESP_LOGE(TAG, "센서 설정 실패");
        return false;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "INA226 초기화 완료");
    return true;
}

bool INA226_Monitor::readStatus(BatteryStatus *status) {
    if (!initialized_) {
        ESP_LOGE(TAG, "INA226이 초기화되지 않았습니다.");
        return false;
    }

    uint16_t bus_voltage_raw, current_raw, power_raw;

    if (!readRegister(INA226_REG_BUS_VOLT, &bus_voltage_raw)) return false;
    if (!readRegister(INA226_REG_CURRENT, &current_raw)) return false;
    if (!readRegister(INA226_REG_POWER, &power_raw)) return false;

    status->voltage = (float)bus_voltage_raw * 1.25f / 1000.0f;
    status->current = (float)((int16_t)current_raw) * current_lsb_ * 1000.0f;
    status->power = (float)power_raw * 25.0f * current_lsb_ * 1000.0f;
    status->is_charging = status->current > 0;

    return true;
}

bool INA226_Monitor::detectSensor() {
    uint8_t addresses[] = {INA226_ADDR_0, INA226_ADDR_1, INA226_ADDR_2, INA226_ADDR_3};
    uint8_t reg = INA226_REG_MANUF_ID;

    for (uint8_t addr : addresses) {
        uint8_t read_buf[2];
        esp_err_t ret = I2CWrapper::write_then_read(I2C_MASTER_NUM, addr, &reg, 1, read_buf, 2, 100); // 감지를 위해 짧은 타임아웃 사용
        if (ret == ESP_OK) {
            uint16_t manuf_id = (read_buf[0] << 8) | read_buf[1];
            if (manuf_id == 0x5449) { // 'TI'
                detected_addr_ = addr;
                return true;
            }
        }
    }
    return false;
}

bool INA226_Monitor::calibrate() {
    current_lsb_ = max_current_ / 32768.0f;
    cal_value_ = (uint16_t)(0.00512f / (current_lsb_ * shunt_resistance_));
    ESP_LOGI(TAG, "캘리브레이션: Current LSB = %f A/bit, Cal Value = %d", current_lsb_, cal_value_);
    return writeRegister(INA226_REG_CALIB, cal_value_);
}

bool INA226_Monitor::configure() {
    uint16_t config = 0x4127; // AVG: 4회, VBUSCT: 1.1ms, VSHCT: 1.1ms, MODE: Shunt and Bus, Continuous
    return writeRegister(INA226_REG_CONFIG, config);
}

bool INA226_Monitor::writeRegister(uint8_t reg, uint16_t value) {
    uint8_t data[3] = {reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
    return I2CWrapper::write_bytes(I2C_MASTER_NUM, detected_addr_, data, 3, I2C_TIMEOUT_MS) == ESP_OK;
}

bool INA226_Monitor::readRegister(uint8_t reg, uint16_t *value) {
    uint8_t read_buf[2];
    esp_err_t ret = I2CWrapper::write_then_read(I2C_MASTER_NUM, detected_addr_, &reg, 1, read_buf, 2, I2C_TIMEOUT_MS);
    if (ret != ESP_OK) return false;
    *value = (read_buf[0] << 8) | read_buf[1];
    return true;
}