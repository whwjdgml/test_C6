/**
 * @file aht20_sensor.h
 * @brief AHT20 온도 및 습도 센서 관리를 위한 클래스 정의
 *
 * 이 파일은 AHT20 센서와의 통신, 데이터 읽기 및 처리를 담당하는
 * AHT20Sensor 클래스를 정의합니다. 센서 초기화, 데이터 수집,
 * 원시 데이터 변환 및 센서 상태 확인 기능을 제공합니다.
 */
#ifndef AHT20_SENSOR_H
#define AHT20_SENSOR_H

#include "sensor_config.h"
#include "sensor_types.h"

/**
 * @class AHT20Sensor
 * @brief AHT20 센서 제어 및 데이터 수집 클래스
 *
 * I2C 통신을 사용하여 AHT20 센서로부터 온도 및 습도 데이터를
 * 읽어오는 기능을 캡슐화합니다.
 */
class AHT20Sensor {
public:
    /**
     * @brief AHT20Sensor 클래스의 생성자
     *
     * 센서가 초기화되지 않은 상태로 객체를 생성합니다.
     */
    AHT20Sensor();
    
    /**
     * @brief 센서를 초기화합니다.
     *
     * 센서에 초기화 명령을 보내고 정상적으로 응답하는지 확인합니다.
     * @return bool 초기화 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool init();
    
    /**
     * @brief 센서로부터 온도 및 습도 데이터를 읽습니다.
     *
     * @param temperature 읽어온 온도 값을 저장할 포인터 (단위: °C)
     * @param humidity 읽어온 습도 값을 저장할 포인터 (단위: %)
     * @return bool 데이터 읽기 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool readData(float *temperature, float *humidity);
    
    /**
     * @brief 센서의 사용 가능 상태를 확인합니다.
     *
     * @return bool 센서가 성공적으로 초기화되었으면 true, 그렇지 않으면 false를 반환합니다.
     */
    bool isAvailable() const { return initialized_; }
    
    /**
     * @brief 센서를 리셋합니다.
     *
     * @return bool 리셋 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool reset();

private:
    bool initialized_; ///< 센서 초기화 상태 저장 변수

    /**
     * @brief 센서에 초기화 명령을 전송합니다.
     *
     * @return bool 명령 전송 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool sendInitCommand();

    /**
     * @brief 센서에 측정 트리거 명령을 전송합니다.
     *
     * @return bool 명령 전송 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool triggerMeasurement();

    /**
     * @brief 센서로부터 원시 데이터를 읽습니다.
     *
     * @param data 읽어온 원시 데이터를 저장할 버퍼 포인터
     * @return bool 데이터 읽기 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool readRawData(uint8_t *data);

    /**
     * @brief 원시 데이터를 실제 온도 및 습도 값으로 변환합니다.
     *
     * @param raw_data 변환할 원시 데이터 버퍼
     * @param temperature 변환된 온도 값을 저장할 포인터
     * @param humidity 변환된 습도 값을 저장할 포인터
     */
    void convertRawData(uint8_t *raw_data, float *temperature, float *humidity);

    /**
     * @brief 읽어온 데이터의 유효성을 검사합니다.
     *
     * @param temperature 검사할 온도 값
     * @param humidity 검사할 습도 값
     * @return bool 데이터가 유효하면 true, 그렇지 않으면 false를 반환합니다.
     */
    bool validateData(float temperature, float humidity);
};

#endif // AHT20_SENSOR_H
