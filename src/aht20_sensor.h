#ifndef AHT20_SENSOR_H
#define AHT20_SENSOR_H

#include "sensor_config.h"
#include "sensor_types.h"

class AHT20Sensor {
public:
    AHT20Sensor();
    
    // 센서 초기화
    bool init();
    
    // 데이터 읽기
    bool readData(float *temperature, float *humidity);
    
    // 센서 상태 확인
    bool isAvailable() const { return initialized_; }
    
    // 센서 리셋
    bool reset();

private:
    bool initialized_;
    
    // 내부 함수들
    bool sendInitCommand();
    bool triggerMeasurement();
    bool readRawData(uint8_t *data);
    void convertRawData(uint8_t *raw_data, float *temperature, float *humidity);
    bool validateData(float temperature, float humidity);
};

#endif // AHT20_SENSOR_H
