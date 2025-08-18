#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <stdint.h>
#include "sensor_types.h"

class SensorManager {
public:
    SensorManager();           // 생성자 선언
    bool init();              // 초기화
    SensorData readAllSensors(); // 데이터 읽기
    bool hasWorkingSensors() const;
    uint8_t getWorkingSensorCount() const;
    void diagnoseSensors(const SensorData &data);

private:
    bool aht20_initialized;
    bool bmp280_initialized;
};

#endif
