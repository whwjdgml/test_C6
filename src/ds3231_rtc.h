#ifndef DS3231_RTC_H
#define DS3231_RTC_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "driver/i2c.h"

// DS3231 I2C 주소 및 레지스터
#define DS3231_ADDR         0x68
#define DS3231_TIME_CAL_ADDR    0x00
#define DS3231_ALARM1_ADDR      0x07
#define DS3231_ALARM2_ADDR      0x0B
#define DS3231_CONTROL_ADDR     0x0E
#define DS3231_STATUS_ADDR      0x0F
#define DS3231_TEMP_MSB_ADDR    0x11
#define DS3231_TEMP_LSB_ADDR    0x12

// 알람 타입
typedef enum {
    DS3231_ALARM1,
    DS3231_ALARM2
} ds3231_alarm_t;

// 시간 구조체
typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t day_of_week;
    uint8_t date;
    uint8_t month;
    uint16_t year;
} rtc_time_t;

// 알람 구조체
typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t date;
    bool match_seconds;
    bool match_minutes;
    bool match_hours;
    bool match_date;
} rtc_alarm_t;

// 적응형 절전 모드
typedef enum {
    ADAPTIVE_POWER_NORMAL,      // 30분 간격
    ADAPTIVE_POWER_SAVE,        // 2시간 간격
    ADAPTIVE_POWER_EMERGENCY    // 4시간 간격
} adaptive_power_mode_t;

// 브로드캐스트 메시지 타입
typedef enum {
    MSG_TIME_SYNC = 0x01,
    MSG_WEATHER_ALERT = 0x02,
    MSG_MODE_CHANGE = 0x03,
    MSG_SYSTEM_STATUS = 0x04
} broadcast_msg_type_t;

// 브로드캐스트 메시지 구조체
typedef struct {
    uint8_t message_type;    // broadcast_msg_type_t
    uint8_t alert_level;     // adaptive_power_mode_t
    uint16_t duration_hours; // 예상 지속 시간
    uint32_t timestamp;      // UNIX 타임스탬프
    uint16_t checksum;       // 메시지 무결성 검증
} __attribute__((packed)) broadcast_message_t;

// DS3231 RTC 클래스
class DS3231_RTC {
public:
    DS3231_RTC();
    ~DS3231_RTC();
    
    // 기본 RTC 기능
    bool init();
    bool setTime(const rtc_time_t& time);
    bool getTime(rtc_time_t& time);
    float getTemperature();
    
    // 알람 기능
    bool setAlarm1(const rtc_alarm_t& alarm);
    bool setAlarm2(const rtc_alarm_t& alarm);
    bool isAlarm1Triggered();
    bool isAlarm2Triggered();
    bool clearAlarm1();
    bool clearAlarm2();
    
    // 절전 모드 스케줄링
    bool setAdaptivePowerMode(adaptive_power_mode_t mode);
    adaptive_power_mode_t getCurrentPowerMode() const;
    uint32_t getNextWakeupTime();
    
    // 브로드캐스트 메시지 기능
    bool enableBroadcastReceive();
    bool checkBroadcastMessage(broadcast_message_t* message);
    
    // 유틸리티 함수
    uint32_t timeToUnixTimestamp(const rtc_time_t& time);
    void unixTimestampToTime(uint32_t timestamp, rtc_time_t& time);
    
private:
    bool initialized_;
    adaptive_power_mode_t current_power_mode_;
    uint32_t last_mode_change_time_;
    
    // I2C 통신
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t* value);
    bool writeMultipleRegisters(uint8_t reg, const uint8_t* data, size_t len);
    bool readMultipleRegisters(uint8_t reg, uint8_t* data, size_t len);
    
    // 유틸리티 함수
    uint8_t decToBcd(uint8_t val);
    uint8_t bcdToDec(uint8_t val);
    uint16_t calculateChecksum(const broadcast_message_t* msg);
    bool validateBroadcastMessage(const broadcast_message_t* msg);
};

#endif // DS3231_RTC_H