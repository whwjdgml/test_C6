/**
 * @file scd41_sensor.h
 * @brief SCD41 CO2, 온도, 습도 센서 관리를 위한 클래스 정의
 *
 * 이 파일은 SCD41 센서와의 통신, 데이터 읽기 및 처리를 담당하는
 * SCD41Sensor 클래스를 정의합니다. 센서 초기화, 주기적 측정 시작/중지,
 * 데이터 수집, 캘리브레이션 제어 등의 기능을 제공합니다.
 */
#ifndef SCD41_SENSOR_H
#define SCD41_SENSOR_H

#include "sensor_config.h"
#include "sensor_types.h"

/**
 * @class SCD41Sensor
 * @brief SCD41 센서 제어 및 데이터 수집 클래스
 *
 * I2C 통신을 사용하여 SCD41 센서로부터 CO2, 온도, 습도 데이터를
 * 읽어오는 기능을 캡슐화합니다.
 */
class SCD41Sensor {
public:
    /**
     * @brief SCD41Sensor 클래스의 생성자
     *
     * 센서가 초기화되지 않은 상태로 객체를 생성합니다.
     */
    SCD41Sensor();
    
    /**
     * @brief 센서를 초기화합니다.
     *
     * 센서를 리셋하고 주기적 측정을 시작합니다.
     * @return bool 초기화 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool init();
    
    /**
     * @brief 센서로부터 CO2, 온도, 습도 데이터를 읽습니다.
     *
     * @param co2 읽어온 CO2 농도 값을 저장할 포인터 (단위: ppm)
     * @param temperature 읽어온 온도 값을 저장할 포인터 (단위: °C)
     * @param humidity 읽어온 습도 값을 저장할 포인터 (단위: %)
     * @return bool 데이터 읽기 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool readData(float *co2, float *temperature, float *humidity);
    
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
    
    /**
     * @brief 자동 자체 캘리브레이션(ASC) 기능을 활성화하거나 비활성화합니다.
     *
     * @param enable 활성화하려면 true, 비활성화하려면 false
     * @return bool 명령 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool setAutomaticSelfCalibration(bool enable);

    /**
     * @brief 현재 자동 자체 캘리브레이션(ASC) 기능의 활성화 상태를 읽어옵니다.
     *
     * @param enabled 현재 활성화 상태를 저장할 포인터
     * @return bool 상태 읽기 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool getAutomaticSelfCalibration(bool* enabled);
    
    /**
     * @brief 알려진 CO2 농도 값을 기준으로 강제 재캘리브레이션(FRC)을 수행합니다.
     *
     * @param co2_reference 기준 CO2 농도 값 (ppm)
     * @return bool 명령 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool performForcedRecalibration(uint16_t co2_reference);

private:
    bool initialized_; ///< 센서 초기화 상태 저장 변수
    
    /**
     * @brief 센서의 주기적 측정을 시작합니다.
     *
     * @return bool 명령 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool startPeriodicMeasurement();

    /**
     * @brief 센서의 주기적 측정을 중지합니다.
     *
     * @return bool 명령 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool stopPeriodicMeasurement();

    /**
     * @brief 새로운 측정 데이터가 준비되었는지 확인합니다.
     *
     * @param data_ready 데이터 준비 상태를 저장할 포인터
     * @return bool 상태 확인 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool getDataReadyStatus(bool *data_ready);

    /**
     * @brief 센서로부터 원시 측정 데이터를 읽습니다.
     *
     * @param co2 원시 CO2 값을 저장할 포인터
     * @param temp 원시 온도 값을 저장할 포인터
     * @param hum 원시 습도 값을 저장할 포인터
     * @return bool 데이터 읽기 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool readMeasurement(uint16_t *co2, uint16_t *temp, uint16_t *hum);

    /**
     * @brief 센서에 인자 없는 명령을 전송합니다.
     *
     * @param command 전송할 16비트 명령어
     * @return bool 명령 전송 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool sendCommand(uint16_t command);

    /**
     * @brief 센서에 인자가 있는 명령을 전송합니다.
     *
     * @param command 전송할 16비트 명령어
     * @param arg 함께 전송할 16비트 인자
     * @return bool 명령 전송 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool sendCommandWithArg(uint16_t command, uint16_t arg);

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
     * @brief 읽어온 데이터의 유효성을 검사합니다.
     *
     * @param co2 검사할 CO2 값
     * @param temperature 검사할 온도 값
     * @param humidity 검사할 습도 값
     * @return bool 데이터가 유효하면 true, 그렇지 않으면 false를 반환합니다.
     */
    bool validateData(float co2, float temperature, float humidity);
};

#endif // SCD41_SENSOR_H