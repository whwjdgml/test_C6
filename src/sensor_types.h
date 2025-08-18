#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H

#include <stdint.h>
#include <stdbool.h>

// 센서 데이터 구조체
struct SensorData {
    float temperature_aht20;
    float humidity_aht20;
    float temperature_bmp280;
    float pressure_bmp280;
    bool aht20_available;
    bool bmp280_available;
    uint64_t timestamp;
};

#endif
