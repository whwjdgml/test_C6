#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <stdint.h>
#include "sensor_types.h"

// Forward declarations
class AHT20Sensor;
class BMP280Sensor;

class SensorManager {
public:
    SensorManager();           // 생성자 선언
    ~SensorManager();          // 소멸자 선언
    bool init();              // 초기화
    SensorData readAllSensors(); // 데이터 읽기
    bool hasWorkingSensors() const;
    uint8_t getWorkingSensorCount() const;
    void diagnoseSensors(const SensorData &data);

private:
    // 센서 객체들
    AHT20Sensor* aht20_sensor;
    BMP280Sensor* bmp280_sensor;
    
    // 초기화 상태
    bool aht20_initialized;
    bool bmp280_initialized;
};

#endif