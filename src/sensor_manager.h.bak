#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <stdint.h>
#include <memory>
#include "sensor_types.h"
#include "sensor_config.h"
#include "ds3231_rtc.h"

// Forward declarations
class AHT20Sensor;
class BMP280Sensor;
class SCD41Sensor;
class SGP40Sensor;
class INA226_Monitor;
class DS3231_RTC;
class NTCSensor;
class BatteryHeater;

class SensorManager {
public:
    SensorManager();           // 생성자 선언
    ~SensorManager();          // 소멸자 선언
    bool init();              // 초기화
    SensorData readAllSensors(); // 데이터 읽기
    bool hasWorkingSensors() const;
    uint8_t getWorkingSensorCount() const;
    void diagnoseSensors(const SensorData &data);

    // 배터리 상태를 읽는 전용 함수
    bool getBatteryStatus(BatteryStatus *status);
    
    // 프로토타입용 추가 기능
    void scanI2CBus();
    bool checkSensorConnections();
    bool validateSensorData(const SensorData &data);
    bool recoverFailedSensors();
    bool testPullupResistors();
    
    // 전력 관리 기능 (향후 구현)
    bool setPowerMode(power_mode_t mode);
    power_mode_t getCurrentPowerMode() const;
    bool initLpCoreI2C();  // LP Core I2C 초기화 (향후 구현)
    
    // DS3231 RTC 기능
    bool initRTC();
    bool setSystemTime(uint16_t year, uint8_t month, uint8_t date, 
                       uint8_t hours, uint8_t minutes, uint8_t seconds);
    bool getSystemTime();
    float getRTCTemperature();
    bool setAdaptivePowerMode(uint8_t mode);
    bool checkBroadcastMessages();
    bool isWakeupByAlarm();
    
    // 배터리 히터 시스템 기능
    bool initBatteryHeater();
    bool updateBatteryHeater();
    float getBatteryTemperature();
    heater_state_t getHeaterState();
    void emergencyHeaterShutdown();
    void printHeaterStatus();
    void printHeaterStatistics();

private:
    // 센서 객체들
    std::unique_ptr<AHT20Sensor> aht20_sensor;
    std::unique_ptr<BMP280Sensor> bmp280_sensor;
    std::unique_ptr<SCD41Sensor> scd41_sensor;
    std::unique_ptr<SGP40Sensor> sgp40_sensor;
    std::unique_ptr<INA226_Monitor> ina226_monitor;
    std::unique_ptr<DS3231_RTC> rtc_module;
    std::unique_ptr<NTCSensor> battery_ntc_sensor;
    std::unique_ptr<BatteryHeater> battery_heater;
    
    // 초기화 상태
    bool aht20_initialized;
    bool bmp280_initialized;
    bool scd41_initialized;
    bool sgp40_initialized;
    bool ina226_initialized;
    bool rtc_initialized;
    bool battery_heater_initialized;
    
    // 전력 관리 상태
    power_mode_t current_power_mode;
    bool lp_core_i2c_initialized;
};

#endif