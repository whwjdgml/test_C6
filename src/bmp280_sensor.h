/**
 * @file bmp280_sensor.h
 * @brief BMP280 기압 및 온도 센서 관리를 위한 클래스 정의
 *
 * 이 파일은 BMP280 센서와의 통신, 데이터 읽기 및 처리를 담당하는
 * BMP280Sensor 클래스를 정의합니다. 센서 초기화, 데이터 수집,
 * 원시 데이터 보정, 고도 계산 및 센서 상태 확인 기능을 제공합니다.
 */
#ifndef BMP280_SENSOR_H
#define BMP280_SENSOR_H

#include "sensor_config.h"
#include "sensor_types.h"

/**
 * @class BMP280Sensor
 * @brief BMP280 센서 제어 및 데이터 수집 클래스
 *
 * I2C 통신을 사용하여 BMP280 센서로부터 온도 및 기압 데이터를
 * 읽어오고, 이를 바탕으로 고도를 계산하는 기능을 캡슐화합니다.
 */
class BMP280Sensor {
public:
    /**
     * @brief BMP280Sensor 클래스의 생성자
     *
     * 센서가 초기화되지 않은 상태로 객체를 생성합니다.
     */
    BMP280Sensor();
    
    /**
     * @brief 센서를 초기화합니다.
     *
     * 센서의 I2C 주소를 감지하고, 캘리브레이션 데이터를 읽어오며,
     * 센서의 측정 설정을 구성합니다.
     * @return bool 초기화 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool init();
    
    /**
     * @brief 센서로부터 온도, 기압, 고도 데이터를 읽습니다.
     *
     * @param temperature 읽어온 온도 값을 저장할 포인터 (단위: °C)
     * @param pressure 읽어온 기압 값을 저장할 포인터 (단위: hPa)
     * @param altitude 계산된 고도 값을 저장할 포인터 (단위: m)
     * @return bool 데이터 읽기 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool readData(float *temperature, float *pressure, float *altitude);
    
    /**
     * @brief 센서의 사용 가능 상태를 확인합니다.
     *
     * @return bool 센서가 성공적으로 초기화되었으면 true, 그렇지 않으면 false를 반환합니다.
     */
    bool isAvailable() const { return initialized_; }

    /**
     * @brief 감지된 센서의 I2C 주소를 반환합니다.
     *
     * @return uint8_t 감지된 I2C 주소. 감지되지 않았으면 0을 반환할 수 있습니다.
     */
    uint8_t getAddress() const { return detected_addr_; }
    
    /**
     * @brief 센서를 리셋합니다.
     *
     * @return bool 리셋 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool reset();

private:
    bool initialized_; ///< 센서 초기화 상태 저장 변수
    uint8_t detected_addr_; ///< 감지된 센서의 I2C 주소
    BMP280CalibData calib_data_; ///< 센서 캘리브레이션 데이터 저장 구조체
    
    /**
     * @brief BMP280 센서의 I2C 주소를 자동으로 감지합니다.
     *
     * @return bool 센서 감지 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool detectSensor();

    /**
     * @brief 센서로부터 캘리브레이션 데이터를 읽어옵니다.
     *
     * @return bool 데이터 읽기 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool readCalibrationData();

    /**
     * @brief 센서의 측정 설정을 구성합니다. (오버샘플링, 필터 등)
     *
     * @return bool 설정 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool configureSensor();

    /**
     * @brief 센서로부터 원시 온도 및 기압 데이터를 읽습니다.
     *
     * @param adc_temp 읽어온 원시 온도 값을 저장할 포인터
     * @param adc_press 읽어온 원시 기압 값을 저장할 포인터
     * @return bool 데이터 읽기 성공 시 true, 실패 시 false를 반환합니다.
     */
    bool readRawData(int32_t *adc_temp, int32_t *adc_press);
    
    /**
     * @brief 원시 온도 값을 보정합니다.
     *
     * @param adc_temp 원시 온도 값
     * @return int32_t 보정된 온도 값. 이 값은 t_fine 계산에 사용됩니다.
     */
    int32_t compensateTemperature(int32_t adc_temp);

    /**
     * @brief 원시 기압 값을 보정합니다.
     *
     * @param adc_press 원시 기압 값
     * @param t_fine 온도 보정 과정에서 계산된 t_fine 값
     * @return uint32_t 보정된 기압 값 (단위: Pa)
     */
    uint32_t compensatePressure(int32_t adc_press, int32_t t_fine);

    /**
     * @brief 기압 값을 사용하여 고도를 계산합니다.
     *
     * @param pressure_hpa 헥토파스칼(hPa) 단위의 기압 값
     * @return float 계산된 고도 값 (단위: m)
     */
    float calculateAltitude(float pressure_hpa);
    
    /**
     * @brief 읽어온 데이터의 유효성을 검사합니다.
     *
     * @param temperature 검사할 온도 값
     * @param pressure 검사할 기압 값
     * @return bool 데이터가 유효하면 true, 그렇지 않으면 false를 반환합니다.
     */
    bool validateData(float temperature, float pressure);
};

#endif // BMP280_SENSOR_H