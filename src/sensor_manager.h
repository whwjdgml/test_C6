/**
 * @file sensor_manager.h
 * @brief 여러 환경 센서들을 총괄 관리하는 클래스 정의
 *
 * 이 파일은 AHT20, BMP280, SCD41, SGP40 등 다양한 센서 객체들을
 * 생성하고, 초기화하며, 주기적으로 데이터를 읽어오는 역할을 하는
 * SensorManager 클래스를 정의합니다.
 */
#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <stdint.h>
#include <memory>
#include "sensor_types.h"

// 클래스 전방 선언
class AHT20Sensor;
class BMP280Sensor;
class SCD41Sensor;
class SGP40Sensor;

/**
 * @class SensorManager
 * @brief 여러 센서들을 통합 관리하는 클래스
 *
 * 다양한 센서들의 초기화, 데이터 수집, 상태 확인을 단일 인터페이스로
 * 제공하여 시스템의 복잡도를 낮춥니다.
 */
class SensorManager {
public:
    /**
     * @brief SensorManager 클래스의 생성자
     *
     * 센서 객체들을 생성하고 초기화 상태를 설정합니다.
     */
    SensorManager();

    /**
     * @brief SensorManager 클래스의 소멸자
     *
     * unique_ptr를 사용하므로 별도의 메모리 해제 코드는 필요하지 않습니다.
     */
    ~SensorManager();

    /**
     * @brief 모든 센서들을 초기화합니다.
     *
     * 각 센서의 init() 함수를 호출하고 성공 여부를 기록합니다.
     * @return bool 하나 이상의 센서가 성공적으로 초기화되면 true를 반환합니다.
     */
    bool init();

    /**
     * @brief 초기화에 성공한 모든 센서로부터 데이터를 읽습니다.
     *
     * @return SensorData 모든 센서의 측정값을 담고 있는 구조체
     */
    SensorData readAllSensors();

    /**
     * @brief 작동 중인(초기화에 성공한) 센서가 하나라도 있는지 확인합니다.
     *
     * @return bool 작동 중인 센서가 있으면 true, 없으면 false
     */
    bool hasWorkingSensors() const;

    /**
     * @brief 작동 중인 센서의 개수를 반환합니다.
     *
     * @return uint8_t 작동 중인 센서의 수
     */
    uint8_t getWorkingSensorCount() const;

    /**
     * @brief 센서 데이터의 유효성을 진단하고 로그를 출력합니다.
     *
     * @param data 진단할 센서 데이터
     */
    void diagnoseSensors(const SensorData &data);

private:
    /// @brief std::unique_ptr를 사용하여 센서 객체들을 관리합니다.
    /// 이렇게 하면 SensorManager 객체가 소멸될 때 자동으로 메모리가 해제됩니다.
    std::unique_ptr<AHT20Sensor> aht20_sensor;
    std::unique_ptr<BMP280Sensor> bmp280_sensor;
    std::unique_ptr<SCD41Sensor> scd41_sensor;
    std::unique_ptr<SGP40Sensor> sgp40_sensor;
    
    /// @brief 각 센서의 초기화 성공 여부를 저장하는 플래그
    bool aht20_initialized;
    bool bmp280_initialized;
    bool scd41_initialized;
    bool sgp40_initialized;
};

#endif // SENSOR_MANAGER_H