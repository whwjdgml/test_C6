#ifndef SCD41_SENSOR_H
#define SCD41_SENSOR_H

#include "sensor_config.h"
#include "sensor_types.h"

class SCD41Sensor {
public:
    SCD41Sensor();
    
    // 센서 초기화
    bool init();
    
    // 데이터 읽기
    bool readData(float *co2, float *temperature, float *humidity);
    
    // 센서 상태 확인
    bool isAvailable() const { return initialized_; }
    
    // 센서 리셋
    bool reset();
    
    // 자동 캘리브레이션 설정
    bool setAutomaticSelfCalibration(bool enable);

    // 자동 캘리브레이션 상태 읽기
    bool getAutomaticSelfCalibration(bool* enabled);
    
    // 강제 재캘리브레이션
    bool performForcedRecalibration(uint16_t co2_reference);

private:
    bool initialized_;
    
    // 내부 함수들
    bool startPeriodicMeasurement();
    bool stopPeriodicMeasurement();
    bool getDataReadyStatus(bool *data_ready);
    bool readMeasurement(uint16_t *co2, uint16_t *temp, uint16_t *hum);
    bool sendCommand(uint16_t command);
    bool sendCommandWithArg(uint16_t command, uint16_t arg);
    bool readResponse(uint8_t *data, size_t len);
    uint8_t calculateCRC8(const uint8_t *data, size_t len);
    bool validateData(float co2, float temperature, float humidity);
};

#endif // SCD41_SENSOR_H