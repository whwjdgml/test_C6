#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H

#include <stdint.h>
#include <stdbool.h>

// BMP280 캘리브레이션 데이터 구조체
struct BMP280CalibData {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    int32_t t_fine;  // 온도 보정용
};

// 센서 데이터 구조체
struct SensorData {
    // AHT20 데이터
    float temperature_aht20;
    float humidity_aht20;
    bool aht20_available;
    
    // BMP280 데이터
    float temperature_bmp280;
    float pressure_bmp280;
    float altitude_bmp280;
    bool bmp280_available;
    
    // SCD41 데이터
    float co2_scd41;
    float temperature_scd41;
    float humidity_scd41;
    bool scd41_available;
    
    // SGP40 데이터
    float voc_index_sgp40;
    bool sgp40_available;
    
    uint64_t timestamp;
};

// 융합 센서 데이터 구조체
struct FusedSensorData {
    float temperature_avg;
    float temperature_diff;
    float humidity;
    float pressure;
    float altitude;
    bool reliable_temperature;
    uint64_t timestamp;
};

#endif