#ifndef BMP280_SENSOR_H
#define BMP280_SENSOR_H

#include "sensor_config.h"
#include "sensor_types.h"

class BMP280Sensor {
public:
    BMP280Sensor();
    
    // 센서 초기화
    bool init();
    
    // 데이터 읽기
    bool readData(float *temperature, float *pressure, float *altitude);
    
    // 센서 상태 확인
    bool isAvailable() const { return initialized_; }
    uint8_t getAddress() const { return detected_addr_; }
    
    // 센서 리셋
    bool reset();

private:
    bool initialized_;
    uint8_t detected_addr_;
    BMP280CalibData calib_data_;
    
    // 내부 함수들
    bool detectSensor();
    bool readCalibrationData();
    bool configureSensor();
    bool readRawData(int32_t *adc_temp, int32_t *adc_press);
    
    // 보정 함수들
    int32_t compensateTemperature(int32_t adc_temp);
    uint32_t compensatePressure(int32_t adc_press, int32_t t_fine);
    float calculateAltitude(float pressure_hpa);
    
    // 유효성 검사
    bool validateData(float temperature, float pressure);
};

#endif // BMP280_SENSOR_H