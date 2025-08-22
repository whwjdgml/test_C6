#include "ds3231_rtc.h"
#include "sensor_config.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include <string.h>
#include <math.h>

static const char *TAG = "DS3231_RTC";

// 각 절전 모드별 측정 간격 (분 단위)
static const uint16_t POWER_MODE_INTERVALS[] = {
    30,   // ADAPTIVE_POWER_NORMAL: 30분
    120,  // ADAPTIVE_POWER_SAVE: 2시간
    240   // ADAPTIVE_POWER_EMERGENCY: 4시간
};

DS3231_RTC::DS3231_RTC() 
    : initialized_(false)
    , current_power_mode_(ADAPTIVE_POWER_NORMAL)
    , last_mode_change_time_(0)
{
}

DS3231_RTC::~DS3231_RTC() {
}

bool DS3231_RTC::init() {
    ESP_LOGI(TAG, "DS3231 RTC 초기화 시작...");
    
    // RTC 연결 테스트
    uint8_t test_reg;
    if (!readRegister(DS3231_CONTROL_ADDR, &test_reg)) {
        ESP_LOGE(TAG, "DS3231 연결 실패 - I2C 통신 오류");
        return false;
    }
    
    // RTC 설정 초기화
    // Control Register: INTCN=1 (알람 인터럽트 활성화)
    uint8_t control_reg = 0x04;
    if (!writeRegister(DS3231_CONTROL_ADDR, control_reg)) {
        ESP_LOGE(TAG, "DS3231 제어 레지스터 설정 실패");
        return false;
    }
    
    // Status Register 초기화 (알람 플래그 클리어)
    uint8_t status_reg = 0x00;
    if (!writeRegister(DS3231_STATUS_ADDR, status_reg)) {
        ESP_LOGE(TAG, "DS3231 상태 레지스터 초기화 실패");
        return false;
    }
    
    // 온도 센서 테스트
    float temp = getTemperature();
    if (temp < -50.0f || temp > 100.0f) {
        ESP_LOGW(TAG, "DS3231 온도 센서 값 비정상: %.2f°C", temp);
    } else {
        ESP_LOGI(TAG, "DS3231 온도 센서: %.2f°C", temp);
    }
    
    initialized_ = true;
    ESP_LOGI(TAG, "✅ DS3231 RTC 초기화 완료");
    
    return true;
}

bool DS3231_RTC::setTime(const rtc_time_t& time) {
    if (!initialized_) {
        ESP_LOGE(TAG, "DS3231이 초기화되지 않음");
        return false;
    }
    
    uint8_t time_data[7];
    time_data[0] = decToBcd(time.seconds);
    time_data[1] = decToBcd(time.minutes);
    time_data[2] = decToBcd(time.hours);
    time_data[3] = decToBcd(time.day_of_week);
    time_data[4] = decToBcd(time.date);
    time_data[5] = decToBcd(time.month);
    time_data[6] = decToBcd(time.year - 2000);
    
    if (!writeMultipleRegisters(DS3231_TIME_CAL_ADDR, time_data, 7)) {
        ESP_LOGE(TAG, "시간 설정 실패");
        return false;
    }
    
    ESP_LOGI(TAG, "시간 설정: %04d-%02d-%02d %02d:%02d:%02d", 
             time.year, time.month, time.date, 
             time.hours, time.minutes, time.seconds);
    
    return true;
}

bool DS3231_RTC::getTime(rtc_time_t& time) {
    if (!initialized_) {
        ESP_LOGE(TAG, "DS3231이 초기화되지 않음");
        return false;
    }
    
    uint8_t time_data[7];
    if (!readMultipleRegisters(DS3231_TIME_CAL_ADDR, time_data, 7)) {
        ESP_LOGE(TAG, "시간 읽기 실패");
        return false;
    }
    
    time.seconds = bcdToDec(time_data[0] & 0x7F);
    time.minutes = bcdToDec(time_data[1] & 0x7F);
    time.hours = bcdToDec(time_data[2] & 0x3F);
    time.day_of_week = bcdToDec(time_data[3] & 0x07);
    time.date = bcdToDec(time_data[4] & 0x3F);
    time.month = bcdToDec(time_data[5] & 0x1F);
    time.year = bcdToDec(time_data[6]) + 2000;
    
    return true;
}

float DS3231_RTC::getTemperature() {
    if (!initialized_) {
        return -999.0f;
    }
    
    uint8_t temp_msb, temp_lsb;
    if (!readRegister(DS3231_TEMP_MSB_ADDR, &temp_msb) ||
        !readRegister(DS3231_TEMP_LSB_ADDR, &temp_lsb)) {
        ESP_LOGE(TAG, "온도 읽기 실패");
        return -999.0f;
    }
    
    // MSB는 정수부, LSB 상위 2비트는 소수부 (0.25°C 해상도)
    int16_t temp_raw = ((int16_t)temp_msb << 8) | temp_lsb;
    float temperature = (float)temp_raw / 256.0f;
    
    return temperature;
}

bool DS3231_RTC::setAdaptivePowerMode(adaptive_power_mode_t mode) {
    if (mode == current_power_mode_) {
        return true; // 이미 같은 모드
    }
    
    ESP_LOGI(TAG, "절전 모드 변경: %d → %d", current_power_mode_, mode);
    
    current_power_mode_ = mode;
    last_mode_change_time_ = esp_log_timestamp();
    
    // 다음 알람 설정
    rtc_time_t current_time;
    if (!getTime(current_time)) {
        ESP_LOGE(TAG, "현재 시간 읽기 실패");
        return false;
    }
    
    // 현재 모드의 간격으로 다음 측정 시간 계산
    uint16_t interval_minutes = POWER_MODE_INTERVALS[mode];
    
    // 다음 측정 시간 계산 (간격에 맞춰 정렬)
    uint16_t next_minutes = ((current_time.minutes / interval_minutes) + 1) * interval_minutes;
    uint8_t next_hour = current_time.hours;
    
    if (next_minutes >= 60) {
        next_minutes -= 60;
        next_hour = (next_hour + 1) % 24;
    }
    
    // 알람1 설정 (시간 정렬 기준)
    rtc_alarm_t alarm;
    alarm.seconds = 0;
    alarm.minutes = next_minutes;
    alarm.hours = next_hour;
    alarm.date = 0;
    alarm.match_seconds = true;
    alarm.match_minutes = true;
    alarm.match_hours = true;
    alarm.match_date = false;
    
    if (!setAlarm1(alarm)) {
        ESP_LOGE(TAG, "알람 설정 실패");
        return false;
    }
    
    ESP_LOGI(TAG, "다음 측정 시간: %02d:%02d (모드: %d, 간격: %d분)", 
             next_hour, next_minutes, mode, interval_minutes);
    
    return true;
}

adaptive_power_mode_t DS3231_RTC::getCurrentPowerMode() const {
    return current_power_mode_;
}

bool DS3231_RTC::setAlarm1(const rtc_alarm_t& alarm) {
    if (!initialized_) {
        return false;
    }
    
    uint8_t alarm_data[4];
    alarm_data[0] = decToBcd(alarm.seconds) | (alarm.match_seconds ? 0x00 : 0x80);
    alarm_data[1] = decToBcd(alarm.minutes) | (alarm.match_minutes ? 0x00 : 0x80);
    alarm_data[2] = decToBcd(alarm.hours) | (alarm.match_hours ? 0x00 : 0x80);
    alarm_data[3] = decToBcd(alarm.date) | (alarm.match_date ? 0x00 : 0x80);
    
    if (!writeMultipleRegisters(DS3231_ALARM1_ADDR, alarm_data, 4)) {
        ESP_LOGE(TAG, "알람1 설정 실패");
        return false;
    }
    
    // 알람1 활성화
    uint8_t control_reg;
    if (!readRegister(DS3231_CONTROL_ADDR, &control_reg)) {
        return false;
    }
    
    control_reg |= 0x01; // A1IE = 1
    return writeRegister(DS3231_CONTROL_ADDR, control_reg);
}

bool DS3231_RTC::isAlarm1Triggered() {
    if (!initialized_) {
        return false;
    }
    
    uint8_t status_reg;
    if (!readRegister(DS3231_STATUS_ADDR, &status_reg)) {
        return false;
    }
    
    return (status_reg & 0x01) != 0; // A1F 플래그 확인
}

bool DS3231_RTC::clearAlarm1() {
    if (!initialized_) {
        return false;
    }
    
    uint8_t status_reg;
    if (!readRegister(DS3231_STATUS_ADDR, &status_reg)) {
        return false;
    }
    
    status_reg &= ~0x01; // A1F 플래그 클리어
    return writeRegister(DS3231_STATUS_ADDR, status_reg);
}

bool DS3231_RTC::enableBroadcastReceive() {
    // ESP-NOW 초기화 (이미 초기화되어 있을 수 있음)
    if (esp_now_init() != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW 초기화 실패");
        return false;
    }
    
    ESP_LOGI(TAG, "브로드캐스트 수신 활성화");
    return true;
}

bool DS3231_RTC::checkBroadcastMessage(broadcast_message_t* message) {
    // 실제 구현에서는 ESP-NOW나 WiFi를 통해 메시지 수신
    // 여기서는 기본 구조만 제공
    
    // TODO: 브로드캐스트 메시지 수신 로직 구현
    // - ESP-NOW 콜백에서 메시지 수신
    // - 메시지 검증 (체크섬, 타임스탬프)
    // - 유효한 경우 message 구조체에 복사
    
    return false; // 현재는 메시지 없음으로 반환
}

// Private 함수들
bool DS3231_RTC::writeRegister(uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    return ret == ESP_OK;
}

bool DS3231_RTC::readRegister(uint8_t reg, uint8_t* value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    return ret == ESP_OK;
}

bool DS3231_RTC::writeMultipleRegisters(uint8_t reg, const uint8_t* data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    for (size_t i = 0; i < len; i++) {
        i2c_master_write_byte(cmd, data[i], true);
    }
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    return ret == ESP_OK;
}

bool DS3231_RTC::readMultipleRegisters(uint8_t reg, uint8_t* data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDR << 1) | I2C_MASTER_READ, true);
    for (size_t i = 0; i < len - 1; i++) {
        i2c_master_read_byte(cmd, &data[i], I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, &data[len - 1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    return ret == ESP_OK;
}

uint8_t DS3231_RTC::decToBcd(uint8_t val) {
    return ((val / 10) << 4) | (val % 10);
}

uint8_t DS3231_RTC::bcdToDec(uint8_t val) {
    return ((val >> 4) * 10) + (val & 0x0F);
}

uint16_t DS3231_RTC::calculateChecksum(const broadcast_message_t* msg) {
    // 간단한 체크섬 계산 (실제로는 CRC16 등 사용 권장)
    uint16_t checksum = 0;
    const uint8_t* data = (const uint8_t*)msg;
    size_t len = sizeof(broadcast_message_t) - sizeof(msg->checksum);
    
    for (size_t i = 0; i < len; i++) {
        checksum += data[i];
    }
    
    return checksum;
}

bool DS3231_RTC::validateBroadcastMessage(const broadcast_message_t* msg) {
    // 체크섬 검증
    uint16_t calculated_checksum = calculateChecksum(msg);
    if (calculated_checksum != msg->checksum) {
        ESP_LOGW(TAG, "브로드캐스트 메시지 체크섬 오류");
        return false;
    }
    
    // 타임스탬프 유효성 검증 (현재 시간 ±24시간 내)
    rtc_time_t current_time;
    if (getTime(current_time)) {
        uint32_t current_timestamp = timeToUnixTimestamp(current_time);
        int32_t time_diff = (int32_t)(msg->timestamp - current_timestamp);
        
        if (abs(time_diff) > 86400) { // 24시간 초과
            ESP_LOGW(TAG, "브로드캐스트 메시지 시간 오류: %d초 차이", time_diff);
            return false;
        }
    }
    
    return true;
}

uint32_t DS3231_RTC::timeToUnixTimestamp(const rtc_time_t& time) {
    struct tm tm_time = {0};
    tm_time.tm_year = time.year - 1900;
    tm_time.tm_mon = time.month - 1;
    tm_time.tm_mday = time.date;
    tm_time.tm_hour = time.hours;
    tm_time.tm_min = time.minutes;
    tm_time.tm_sec = time.seconds;
    
    return (uint32_t)mktime(&tm_time);
}

void DS3231_RTC::unixTimestampToTime(uint32_t timestamp, rtc_time_t& time) {
    time_t t = timestamp;
    struct tm* tm_time = localtime(&t);
    
    time.year = tm_time->tm_year + 1900;
    time.month = tm_time->tm_mon + 1;
    time.date = tm_time->tm_mday;
    time.hours = tm_time->tm_hour;
    time.minutes = tm_time->tm_min;
    time.seconds = tm_time->tm_sec;
    time.day_of_week = tm_time->tm_wday;
}