/**
 * @file sensor_manager.cpp
 * @brief 여러 환경 센서들을 총괄 관리하는 클래스의 구현부
 *
 * SensorManager 클래스의 멤버 함수들을 실제로 구현합니다.
 * I2C 버스 초기화, 각 센서의 초기화 시도, 주기적인 데이터 수집,
 * 센서 상태 진단 등의 로직이 포함되어 있습니다.
 */
#include "sensor_manager.h"
#include "sensor_config.h"
#include "aht20_sensor.h"
#include "bmp280_sensor.h"
#include "scd41_sensor.h"
#include "sgp40_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"

// 로그 태그
static const char *TAG = "SENSOR_MGR";

#define I2C_MASTER_NUM          I2C_NUM_0   ///< I2C 마스터 번호
#define I2C_MASTER_TIMEOUT_MS   1000        ///< I2C 통신 타임아웃 (ms)

/**
 * @brief SensorManager 생성자
 *
 * 모든 센서 객체를 std::make_unique를 사용하여 동적으로 생성하고,
 * 초기화 상태 플래그를 false로 설정합니다.
 */
SensorManager::SensorManager()
    : aht20_initialized(false), bmp280_initialized(false),
      scd41_initialized(false), sgp40_initialized(false)
{
    aht20_sensor = std::make_unique<AHT20Sensor>();
    bmp280_sensor = std::make_unique<BMP280Sensor>();
    scd41_sensor = std::make_unique<SCD41Sensor>();
    sgp40_sensor = std::make_unique<SGP40Sensor>();
}

/**
 * @brief SensorManager 소멸자
 *
 * std::unique_ptr가 관리하는 센서 객체들의 메모리는 자동으로 해제됩니다.
 */
SensorManager::~SensorManager() = default;

/**
 * @brief I2C 버스를 설정하고 모든 센서를 순차적으로 초기화합니다.
 *
 * 각 센서의 init()을 호출하고, 성공 여부에 따라 _initialized 플래그를 업데이트합니다.
 * @return bool 하나 이상의 센서가 성공적으로 초기화되면 true를 반환합니다.
 */
bool SensorManager::init() {
    ESP_LOGI(TAG, "SensorManager 초기화 시작...");
    ESP_LOGI(TAG, "I2C 핀: SDA=GPIO%d, SCL=GPIO%d", I2C_SDA_PIN, I2C_SCL_PIN);
    
    // I2C 마스터 설정
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_PIN;
    conf.scl_io_num = I2C_SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000; // 100KHz
    conf.clk_flags = 0;
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config 실패: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install 실패: %s", esp_err_to_name(ret));
        return false;
    }
    
    ESP_LOGI(TAG, "I2C 초기화 성공");
    vTaskDelay(pdMS_TO_TICKS(200)); // I2C 버스 안정화 대기
    
    // AHT20 초기화 시도
    ESP_LOGI(TAG, "AHT20 초기화 중...");
    if (aht20_sensor && aht20_sensor->init()) {
        aht20_initialized = true;
        ESP_LOGI(TAG, "AHT20 초기화 성공");
    } else {
        ESP_LOGW(TAG, "AHT20 초기화 실패");
    }
    
    // BMP280 초기화 시도
    ESP_LOGI(TAG, "BMP280 초기화 중...");
    if (bmp280_sensor && bmp280_sensor->init()) {
        bmp280_initialized = true;
        ESP_LOGI(TAG, "BMP280 초기화 성공 (주소: 0x%02X)", bmp280_sensor->getAddress());
    } else {
        ESP_LOGW(TAG, "BMP280 초기화 실패");
    }
    
    // SCD41 초기화 시도
    ESP_LOGI(TAG, "SCD41 초기화 중...");
    if (scd41_sensor && scd41_sensor->init()) {
        scd41_initialized = true;
        ESP_LOGI(TAG, "SCD41 초기화 성공");
    } else {
        ESP_LOGW(TAG, "SCD41 초기화 실패");
    }
    
    // SGP40 초기화 시도
    ESP_LOGI(TAG, "SGP40 초기화 중...");
    if (sgp40_sensor && sgp40_sensor->init()) {
        sgp40_initialized = true;
        ESP_LOGI(TAG, "SGP40 초기화 성공");
    } else {
        ESP_LOGW(TAG, "SGP40 초기화 실패");
    }
    
    // 최종 결과 요약
    uint8_t working_sensors = getWorkingSensorCount();
    if (working_sensors > 0) {
        ESP_LOGI(TAG, "SensorManager 초기화 완료! 작동중인 센서: %d", working_sensors);
        if (aht20_initialized) ESP_LOGI(TAG, "   - AHT20: 온도/습도 센서");
        if (bmp280_initialized) ESP_LOGI(TAG, "   - BMP280: 기압/온도 센서");
        if (scd41_initialized) ESP_LOGI(TAG, "   - SCD41: CO2/온도/습도 센서");
        if (sgp40_initialized) ESP_LOGI(TAG, "   - SGP40: VOC 인덱스 센서");
    } else {
        ESP_LOGE(TAG, "SensorManager 초기화 실패 - 작동하는 센서 없음");
    }
    
    return (working_sensors > 0);
}

/**
 * @brief 모든 활성화된 센서로부터 데이터를 읽어 SensorData 구조체에 채웁니다.
 *
 * 각 센서의 가용성을 확인하고, 가능한 경우에만 데이터를 읽습니다.
 * SGP40의 경우, 다른 센서(SCD41 또는 AHT20)의 온도/습도 데이터를 보상값으로 사용합니다.
 * @return SensorData 수집된 센서 데이터
 */
SensorData SensorManager::readAllSensors() {
    SensorData data = {};
    data.timestamp = esp_timer_get_time() / 1000; // 밀리초 단위 타임스탬프
    data.aht20_available = false;
    data.bmp280_available = false;
    data.scd41_available = false;
    data.sgp40_available = false;
    
    // AHT20 데이터 읽기
    if (aht20_initialized && aht20_sensor) {
        if (aht20_sensor->readData(&data.temperature_aht20, &data.humidity_aht20)) {
            data.aht20_available = true;
        } else {
            ESP_LOGW(TAG, "AHT20 데이터 읽기 실패");
        }
    }
    
    // BMP280 데이터 읽기
    if (bmp280_initialized && bmp280_sensor) {
        if (bmp280_sensor->readData(&data.temperature_bmp280, &data.pressure_bmp280, &data.altitude_bmp280)) {
            data.bmp280_available = true;
        } else {
            ESP_LOGW(TAG, "BMP280 데이터 읽기 실패");
        }
    }
    
    // SCD41 데이터 읽기
    if (scd41_initialized && scd41_sensor) {
        if (scd41_sensor->readData(&data.co2_scd41, &data.temperature_scd41, &data.humidity_scd41)) {
            data.scd41_available = true;
        } else {
            ESP_LOGW(TAG, "SCD41 데이터 읽기 실패");
        }
    }
    
    // SGP40 데이터 읽기 (온도/습도 보정 필요)
    if (sgp40_initialized && sgp40_sensor) {
        float comp_temp, comp_hum;
        bool comp_available = false;
        const char* comp_source = "N/A";

        // 보상 데이터 소스 우선순위: 1. SCD41, 2. AHT20
        if (data.scd41_available) {
            comp_temp = data.temperature_scd41;
            comp_hum = data.humidity_scd41;
            comp_available = true;
            comp_source = "SCD41";
        } else if (data.aht20_available) {
            comp_temp = data.temperature_aht20;
            comp_hum = data.humidity_aht20;
            comp_available = true;
            comp_source = "AHT20";
        }

        if (comp_available) {
            if (sgp40_sensor->readData(&data.voc_index_sgp40, comp_hum, comp_temp)) {
                data.sgp40_available = true;
                ESP_LOGD(TAG, "SGP40 읽기 성공: VOC=%.0f (보상 소스: %s T:%.1f, H:%.1f)", data.voc_index_sgp40, comp_source, comp_temp, comp_hum);
            } else {
                ESP_LOGW(TAG, "SGP40 데이터 읽기 실패");
            }
        } else {
            ESP_LOGW(TAG, "SGP40 읽기 건너뜀: 보상 데이터 없음");
        }
    }
    
    return data;
}

/**
 * @brief 작동 중인 센서가 있는지 확인합니다.
 * @return bool 하나라도 있으면 true
 */
bool SensorManager::hasWorkingSensors() const {
    return (aht20_initialized || bmp280_initialized || scd41_initialized || sgp40_initialized);
}

/**
 * @brief 작동 중인 센서의 개수를 반환합니다.
 * @return uint8_t 작동 중인 센서 수
 */
uint8_t SensorManager::getWorkingSensorCount() const {
    uint8_t count = 0;
    if (aht20_initialized) count++;
    if (bmp280_initialized) count++;
    if (scd41_initialized) count++;
    if (sgp40_initialized) count++;
    return count;
}

/**
 * @brief 수집된 센서 데이터를 기반으로 시스템 상태를 진단하고 로그를 출력합니다.
 * @param data 진단할 센서 데이터
 */
void SensorManager::diagnoseSensors(const SensorData &data) {
    ESP_LOGI(TAG, "--- 센서 진단 시작 ---");
    
    // 각 센서의 온라인/오프라인 상태 출력
    ESP_LOGI(TAG, "AHT20: %s", data.aht20_available ? "온라인 (온도/습도)" : "오프라인");
    ESP_LOGI(TAG, "BMP280: %s", data.bmp280_available ? "온라인 (기압/온도)" : "오프라인");
    ESP_LOGI(TAG, "SCD41: %s", data.scd41_available ? "온라인 (CO2/온도/습도)" : "오프라인");
    ESP_LOGI(TAG, "SGP40: %s", data.sgp40_available ? "온라인 (VOC 인덱스)" : "오프라인");
    ESP_LOGI(TAG, "총 작동 센서: %d", this->getWorkingSensorCount());
    
    // 개별 센서 값 출력
    if (data.aht20_available) ESP_LOGI(TAG, "AHT20 값: 온도=%.2f°C, 습도=%.2f%%", data.temperature_aht20, data.humidity_aht20);
    if (data.bmp280_available) ESP_LOGI(TAG, "BMP280 값: 온도=%.2f°C, 기압=%.2fhPa, 고도=%.2fm", data.temperature_bmp280, data.pressure_bmp280, data.altitude_bmp280);
    if (data.scd41_available) {
        ESP_LOGI(TAG, "SCD41 값: CO2=%.0fppm, 온도=%.2f°C, 습도=%.2f%%", data.co2_scd41, data.temperature_scd41, data.humidity_scd41);
        bool asc_enabled;
        if (scd41_sensor && scd41_sensor->getAutomaticSelfCalibration(&asc_enabled)) {
            ESP_LOGI(TAG, "SCD41 자동 캘리브레이션(ASC): %s", asc_enabled ? "활성화됨" : "비활성화됨");
        }
    }
    if (data.sgp40_available) ESP_LOGI(TAG, "SGP40 값: VOC 인덱스=%.0f", data.voc_index_sgp40);
    
    // 여러 센서 간 데이터 비교 분석 (온도)
    int temp_sensors = (data.aht20_available ? 1:0) + (data.bmp280_available ? 1:0) + (data.scd41_available ? 1:0);
    if (temp_sensors >= 2) {
        float temp_total = 0;
        if(data.aht20_available) temp_total += data.temperature_aht20;
        if(data.bmp280_available) temp_total += data.temperature_bmp280;
        if(data.scd41_available) temp_total += data.temperature_scd41;
        float avg_temp = temp_total / temp_sensors;
        ESP_LOGI(TAG, "평균 온도: %.2f°C (%d개 센서 기반)", avg_temp, temp_sensors);
    }
    
    // 환경 지표 분석 (CO2, VOC)
    if (data.scd41_available) {
        const char* co2_level = (data.co2_scd41 < 800) ? "좋음" : (data.co2_scd41 < 1000) ? "보통" : (data.co2_scd41 < 1500) ? "나쁨" : "매우 나쁨";
        ESP_LOGI(TAG, "CO2 수준: %s (%.0fppm)", co2_level, data.co2_scd41);
    }
    if (data.sgp40_available) {
        const char* voc_level = (data.voc_index_sgp40 < 100) ? "최상" : (data.voc_index_sgp40 < 200) ? "좋음" : (data.voc_index_sgp40 < 300) ? "보통" : "나쁨";
        ESP_LOGI(TAG, "VOC 지수: %s (%.0f)", voc_level, data.voc_index_sgp40);
    }
    ESP_LOGI(TAG, "--- 센서 진단 종료 ---");
}