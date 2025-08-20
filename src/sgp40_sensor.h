#ifndef SGP40_SENSOR_H
#define SGP40_SENSOR_H

#include "sensor_config.h"
#include "sensor_types.h"
#include "voc_algorithm.h"

class SGP40Sensor {
public:
    SGP40Sensor();

    // 센서 초기화
    bool init();

    // 데이터 읽기 (온도/습도 보정 필요)
    bool readData(float *voc_index, float humidity, float temperature);

    // 센서 상태 확인
    bool isAvailable() const { return initialized_; }

private:
    bool initialized_;
    GasIndexAlgorithmParams voc_params_;

    // 내부 함수들
    bool executeSelfTest();
    bool sendCommand(uint16_t command, uint16_t arg1, uint16_t arg2);
    bool readResponse(uint8_t *data, size_t len);
    uint8_t calculateCRC8(const uint8_t *data, size_t len);
    bool validateData(float voc_index);
};

#endif // SGP40_SENSOR_H