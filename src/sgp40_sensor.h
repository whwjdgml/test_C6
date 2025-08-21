/**
 * @file sgp40_sensor.h
 * @brief SGP40 VOC(휘발성 유기 화합물) 센서 관리를 위한 클래스 정의
 *
 * 이 파일은 SGP40 센서와의 통신, 데이터 읽기 및 VOC 인덱스 계산을
 * 담당하는 SGP40Sensor 클래스를 정의합니다. 센서 초기화, 자체 테스트,
 * 온도/습도 보정을 통한 측정, VOC 알고리즘 연동 기능을 제공합니다.
 */
#ifndef SGP40_SENSOR_H
#define SGP40_SENSOR_H

#include "sensor_config.h"
#include "sensor_types.h"
#include "voc_algorithm.h"

/**
 * @class SGP40Sensor
 * @brief SGP40 VOC 센서 제어 및 VOC 인덱스 계산 클래스
 *
 * I2C 통신을 사용하여 SGP40 센서로부터 원시 데이터를 읽고,
 * Sensirion의 VOC 알고리즘을 사용하여 VOC 인덱스를 계산하는 기능을 캡슐화합니다.
 * 정확한 측정을 위해 외부의 온도 및 습도 데이터가 필요합니다.
 */
class SGP40Sensor {
public:
    /**
     * @brief SGP40Sensor 클래스의 생성자
     *
     * 센서가 초기화되지 않은 상태로 객체를 생성하고, VOC 알고리즘을 초기화합니다.
     */
    SGP40Sensor();

    /**
     * @brief 센서를 초기화합니다.
     *
     * 센서의 자체 테스트를 수행하여 정상 작동 여부를 확인합니다.
     * @return bool 초기화(자체 테스트) 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool init();

    /**
     * @brief 센서로부터 VOC 인덱스를 읽습니다.
     *
     * 현재 온도와 습도 값을 이용하여 측정값을 보정하고, VOC 알고리즘을 통해
     * 최종 VOC 인덱스를 계산합니다.
     * @param voc_index 계산된 VOC 인덱스(0-500)를 저장할 포인터
     * @param humidity 현재 상대 습도 (%RH)
     * @param temperature 현재 온도 (°C)
     * @return bool 데이터 읽기 및 계산 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool readData(float *voc_index, float humidity, float temperature);

    /**
     * @brief 센서의 사용 가능 상태를 확인합니다.
     *
     * @return bool 센서가 성공적으로 초기화되었으면 true, 그렇지 않으면 false를 반환합니다.
     */
    bool isAvailable() const { return initialized_; }

private:
    bool initialized_; ///< 센서 초기화 상태 저장 변수
    GasIndexAlgorithmParams voc_params_; ///< VOC 인덱스 계산 알고리즘 파라미터

    /**
     * @brief 센서의 자체 테스트를 실행합니다.
     *
     * @return bool 자체 테스트 통과 시 true, 실패 시 false를 반환합니다.
     */
    bool executeSelfTest();

    /**
     * @brief 센서에 측정 명령을 전송합니다.
     *
     * @param command 전송할 16비트 명령어
     * @param arg1 상대 습도 보정값
     * @param arg2 온도 보정값
     * @return bool 명령 전송 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool sendCommand(uint16_t command, uint16_t arg1, uint16_t arg2);

    /**
     * @brief 센서로부터 응답 데이터를 읽습니다.
     *
     * @param data 읽은 데이터를 저장할 버퍼
     * @param len 읽을 데이터의 길이 (바이트)
     * @return bool 데이터 읽기 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool readResponse(uint8_t *data, size_t len);

    /**
     * @brief Sensirion I2C 통신용 CRC-8 체크섬을 계산합니다.
     *
     * @param data CRC를 계산할 데이터 배열
     * @param len 데이터의 길이 (바이트)
     * @return uint8_t 계산된 CRC 값
     */
    uint8_t calculateCRC8(const uint8_t *data, size_t len);

    /**
     * @brief 계산된 VOC 인덱스의 유효성을 검사합니다.
     *
     * @param voc_index 검사할 VOC 인덱스 값
     * @return bool 데이터가 유효하면 true, 그렇지 않으면 false를 반환합니다.
     */
    bool validateData(float voc_index);
};

#endif // SGP40_SENSOR_H