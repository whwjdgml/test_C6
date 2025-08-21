#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <stdint.h>
#include <memory>
#include "sensor_types.h"

// Forward declarations
class AHT20Sensor;
class BMP280Sensor;
class SCD41Sensor;
class SGP40Sensor;
class INA226_Monitor;

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

private:
    // 센서 객체들
    std::unique_ptr<AHT20Sensor> aht20_sensor;
    std::unique_ptr<BMP280Sensor> bmp280_sensor;
    std::unique_ptr<SCD41Sensor> scd41_sensor;
    std::unique_ptr<SGP40Sensor> sgp40_sensor;
    std::unique_ptr<INA226_Monitor> ina226_monitor;
    
    // 초기화 상태
    bool aht20_initialized;
    bool bmp280_initialized;
    bool scd41_initialized;
    bool sgp40_initialized;
    bool ina226_initialized;
};

#endif