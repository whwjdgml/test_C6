#include "sensor_manager.h"
#include "sensor_config.h"
#include "aht20_sensor.h"
#include "bmp280_sensor.h"
#include "scd41_sensor.h"
#include "sgp40_sensor.h"
#include "ina226_monitor.h"
#include "ds3231_rtc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "SENSOR_MGR";

#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_TIMEOUT_MS   1000

// --- 사용자 설정: INA226 파라미터 ---
// 사용 중인 션트 저항의 값 (Ohm). 실제 하드웨어에 맞게 수정해야 합니다.
#define INA226_SHUNT_RESISTANCE  0.05f //측정후 수정할것!

// 예상되는 최대 전류 (Ampere). 이 값을 기준으로 측정 정밀도가 결정됩니다.
#define INA226_MAX_CURRENT       1.0f 

SensorManager::SensorManager() : aht20_sensor(nullptr), bmp280_sensor(nullptr), scd41_sensor(nullptr), 
                                 aht20_initialized(false), bmp280_initialized(false), scd41_initialized(false), sgp40_initialized(false), ina226_initialized(false), rtc_initialized(false),
                                 current_power_mode(POWER_MODE_NORMAL), lp_core_i2c_initialized(false) 
{
    aht20_sensor = std::make_unique<AHT20Sensor>();
    bmp280_sensor = std::make_unique<BMP280Sensor>();
    scd41_sensor = std::make_unique<SCD41Sensor>();
    sgp40_sensor = std::make_unique<SGP40Sensor>();
    ina226_monitor = std::make_unique<INA226_Monitor>(INA226_SHUNT_RESISTANCE, INA226_MAX_CURRENT);
    rtc_module = std::make_unique<DS3231_RTC>();
}

SensorManager::~SensorManager() = default; // 스마트 포인터가 자동으로 메모리를 해제합니다.

bool SensorManager::init() {
    ESP_LOGI(TAG, "SensorManager initializing...");
    ESP_LOGI(TAG, "I2C pins: SDA=GPIO%d, SCL=GPIO%d", I2C_SDA_PIN, I2C_SCL_PIN);
    
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_PIN;
    conf.scl_io_num = I2C_SCL_PIN;
    // HP Core I2C 설정 (High-Power 메인 코어)
    // - 현재 GPIO22(SDA)/GPIO23(SCL) 사용 (Xiao ESP32C6 D4/D5 핀)
    // - 센서 모듈에 풀업저항 확인됨 → 외부 풀업 사용
    // - LP Core I2C는 GPIO6(SDA)/GPIO7(SCL) 고정 (저전력 모드용)
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;  // 외부 풀업 사용 (센서 모듈에 있음)
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;  // 외부 풀업 사용 (센서 모듈에 있음)
    conf.master.clk_speed = 400000; // 400kHz Fast Mode (HP Core + 외부 풀업으로 안정적)
    conf.clk_flags = 0;
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    ESP_LOGI(TAG, "I2C initialized successfully");
    vTaskDelay(pdMS_TO_TICKS(200));
    
    auto init_and_log = [this](const char* name, auto& sensor, bool& flag, auto&& success_log_action) {
        ESP_LOGI(TAG, "Initializing %s...", name);
        if (sensor && sensor->init()) {
            flag = true;
            success_log_action();
        } else {
            ESP_LOGW(TAG, "%s initialization failed", name);
        }
    };

    init_and_log("AHT20", aht20_sensor, aht20_initialized,
                 [](){ ESP_LOGI(TAG, "AHT20 initialization successful"); });

    init_and_log("BMP280", bmp280_sensor, bmp280_initialized,
                 [this](){ ESP_LOGI(TAG, "BMP280 initialization successful (addr: 0x%02X)", bmp280_sensor->getAddress()); });

    init_and_log("SCD41", scd41_sensor, scd41_initialized,
                 [](){ ESP_LOGI(TAG, "SCD41 initialization successful"); });

    init_and_log("SGP40", sgp40_sensor, sgp40_initialized,
                 [](){ ESP_LOGI(TAG, "SGP40 initialization successful"); });
    
    // Initialize INA226
    ESP_LOGI(TAG, "Initializing INA226 Battery Monitor...");
    if (ina226_monitor && ina226_monitor->init()) {
        ina226_initialized = true;
        ESP_LOGI(TAG, "INA226 initialization successful (addr: 0x%02X)", ina226_monitor->getAddress());
    } else {
        ESP_LOGW(TAG, "INA226 initialization failed");
    }
    
    uint8_t working_sensors = getWorkingSensorCount();
    if (working_sensors > 0 || ina226_initialized) {
        ESP_LOGI(TAG, "SensorManager initialization complete! Working sensors: %d", working_sensors);
        if (aht20_initialized)  ESP_LOGI(TAG, "   - AHT20: temp/humidity sensor");
        if (bmp280_initialized) ESP_LOGI(TAG, "   - BMP280: pressure/temp sensor");
        if (scd41_initialized)  ESP_LOGI(TAG, "   - SCD41: CO2/temp/humidity sensor");
        if (sgp40_initialized)  ESP_LOGI(TAG, "   - SGP40: VOC Index sensor");
    } else {
        ESP_LOGE(TAG, "SensorManager initialization failed - no working sensors");
    }
    
    return (working_sensors > 0 || ina226_initialized);
}

SensorData SensorManager::readAllSensors() {
    SensorData data = {};
    data.timestamp = esp_timer_get_time() / 1000;
    data.aht20_available = false;
    data.bmp280_available = false;
    data.scd41_available = false;
    data.sgp40_available = false;
    
    // Read AHT20 data
    if (aht20_initialized && aht20_sensor) {
        float temp, hum;
        if (aht20_sensor->readData(&temp, &hum)) {
            data.temperature_aht20 = temp;
            data.humidity_aht20 = hum;
            data.aht20_available = true;
            ESP_LOGD(TAG, "AHT20 read success: temp=%.2f°C, hum=%.2f%%", temp, hum);
        } else {
            ESP_LOGW(TAG, "AHT20 data read failed");
        }
    }
    
    // Read BMP280 data
    if (bmp280_initialized && bmp280_sensor) {
        float temp, press, alt;
        if (bmp280_sensor->readData(&temp, &press, &alt)) {
            data.temperature_bmp280 = temp;
            data.pressure_bmp280 = press;
            data.altitude_bmp280 = alt;
            data.bmp280_available = true;
            ESP_LOGD(TAG, "BMP280 read success: temp=%.2f°C, press=%.2fhPa, alt=%.2fm", temp, press, alt);
        } else {
            ESP_LOGW(TAG, "BMP280 data read failed");
        }
    }
    
    // Read SCD41 data
    if (scd41_initialized && scd41_sensor) {
        float co2, temp, hum;
        if (scd41_sensor->readData(&co2, &temp, &hum)) {
            data.co2_scd41 = co2;
            data.temperature_scd41 = temp;
            data.humidity_scd41 = hum;
            data.scd41_available = true;
            ESP_LOGD(TAG, "SCD41 read success: CO2=%.0fppm, temp=%.2f°C, hum=%.2f%%", co2, temp, hum);
        } else {
            ESP_LOGW(TAG, "SCD41 data read failed");
        }
    }
    
    // Read SGP40 data (needs temp/hum for compensation)
    if (sgp40_initialized && sgp40_sensor) {
        float comp_temp, comp_hum;
        bool comp_available = false;
        const char* comp_source = "N/A";

        // Prioritize SCD41 for compensation data, fallback to AHT20
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
            float voc_index;
            if (sgp40_sensor->readData(&voc_index, comp_hum, comp_temp)) {
                data.voc_index_sgp40 = voc_index;
                data.sgp40_available = true;
                ESP_LOGD(TAG, "SGP40 read success: VOC Index=%.0f (comp from %s T:%.1f, H:%.1f)", voc_index, comp_source, comp_temp, comp_hum);
            } else {
                ESP_LOGW(TAG, "SGP40 data read failed");
            }
        } else {
            ESP_LOGW(TAG, "SGP40 read skipped: no compensation data available");
        }
    }
    
    return data;
}

bool SensorManager::getBatteryStatus(BatteryStatus *status) {
    if (ina226_initialized && ina226_monitor) {
        return ina226_monitor->readStatus(status);
    }
    return false;
}

bool SensorManager::hasWorkingSensors() const {
    return (aht20_initialized || bmp280_initialized || scd41_initialized || sgp40_initialized);
}

uint8_t SensorManager::getWorkingSensorCount() const {
    uint8_t count = 0;
    if (aht20_initialized) count++;
    if (bmp280_initialized) count++;
    if (scd41_initialized) count++;
    if (sgp40_initialized) count++;
    return count;
}

void SensorManager::diagnoseSensors(const SensorData &data) {
    ESP_LOGI(TAG, "Sensor diagnostics:");
    ESP_LOGI(TAG, "AHT20: %s %s", 
             data.aht20_available ? "OK" : "OFFLINE",
             data.aht20_available ? "(temp/humidity)" : "");
    
    ESP_LOGI(TAG, "BMP280: %s %s", 
             data.bmp280_available ? "OK" : "OFFLINE",
             data.bmp280_available ? "(pressure/temp)" : "");
    
    ESP_LOGI(TAG, "SCD41: %s %s", 
             data.scd41_available ? "OK" : "OFFLINE",
             data.scd41_available ? "(CO2/temp/humidity)" : "");
    
    ESP_LOGI(TAG, "SGP40: %s %s", 
             data.sgp40_available ? "OK" : "OFFLINE",
             data.sgp40_available ? "(VOC Index)" : "");
    
    ESP_LOGI(TAG, "INA226: %s %s", 
             ina226_initialized ? "OK" : "OFFLINE",
             ina226_initialized ? "(Battery Monitor)" : "");
    
    ESP_LOGI(TAG, "DS3231: %s %s", 
             rtc_initialized ? "OK" : "OFFLINE",
             rtc_initialized ? "(RTC + Temperature)" : "");
    
    ESP_LOGI(TAG, "Total working sensors: %d", this->getWorkingSensorCount());
    
    if (data.aht20_available) {
        ESP_LOGI(TAG, "AHT20 values: temp=%.2f°C, hum=%.2f%%", 
                data.temperature_aht20, data.humidity_aht20);
    }
    
    if (data.bmp280_available) {
        ESP_LOGI(TAG, "BMP280 values: temp=%.2f°C, press=%.2fhPa, alt=%.2fm", 
                data.temperature_bmp280, data.pressure_bmp280, data.altitude_bmp280);
    }
    
    if (data.scd41_available) {
        ESP_LOGI(TAG, "SCD41 values: CO2=%.0fppm, temp=%.2f°C, hum=%.2f%%", 
                data.co2_scd41, data.temperature_scd41, data.humidity_scd41);
        
        bool asc_enabled;
        if (scd41_sensor && scd41_sensor->getAutomaticSelfCalibration(&asc_enabled)) {
            ESP_LOGI(TAG, "SCD41 Auto-Calibration (ASC): %s", asc_enabled ? "ENABLED" : "DISABLED (Low Power Mode)");
        }
    }

    if (data.sgp40_available) {
        ESP_LOGI(TAG, "SGP40 values: VOC Index=%.0f", 
                data.voc_index_sgp40);
    }
    
    // INA226 배터리 상태 표시
    if (ina226_initialized) {
        BatteryStatus battery_status;
        if (getBatteryStatus(&battery_status)) {
            ESP_LOGI(TAG, "INA226 values: %.3fV, %.2fmA, %.2fmW [%s]",
                     battery_status.voltage, battery_status.current, 
                     battery_status.power, battery_status.is_charging ? "CHARGING" : "DISCHARGING");
        } else {
            ESP_LOGW(TAG, "INA226 data read failed");
        }
    }
    
    // DS3231 RTC 온도 표시
    if (rtc_initialized) {
        float rtc_temp = getRTCTemperature();
        if (rtc_temp > -50.0f && rtc_temp < 100.0f) {
            ESP_LOGI(TAG, "DS3231 values: temp=%.2f°C", rtc_temp);
        } else {
            ESP_LOGW(TAG, "DS3231 temperature read failed");
        }
    }
    
    // Temperature comparison analysis
    int temp_sensors = 0;
    float temp_total = 0;
    float rtc_temp = 0;
    bool rtc_temp_valid = false;
    
    if (data.aht20_available) {
        temp_total += data.temperature_aht20;
        temp_sensors++;
    }
    
    if (data.bmp280_available) {
        temp_total += data.temperature_bmp280;
        temp_sensors++;
    }
    
    if (data.scd41_available) {
        temp_total += data.temperature_scd41;
        temp_sensors++;
    }
    
    // DS3231 온도도 포함 (별도 변수로 관리)
    if (rtc_initialized) {
        rtc_temp = getRTCTemperature();
        if (rtc_temp > -50.0f && rtc_temp < 100.0f) {
            rtc_temp_valid = true;
        }
    }
    
    if (temp_sensors >= 2) {
        float avg_temp = temp_total / temp_sensors;
        ESP_LOGI(TAG, "Temperature average: %.2f°C (%d sensors)", avg_temp, temp_sensors);
        
        if (data.aht20_available && data.bmp280_available) {
            float diff_aht_bmp = data.temperature_aht20 - data.temperature_bmp280;
            ESP_LOGI(TAG, "Temp diff (AHT20-BMP280): %.2f°C", diff_aht_bmp);
        }
        
        if (data.aht20_available && data.scd41_available) {
            float diff_aht_scd = data.temperature_aht20 - data.temperature_scd41;
            ESP_LOGI(TAG, "Temp diff (AHT20-SCD41): %.2f°C", diff_aht_scd);
        }
        
        if (data.bmp280_available && data.scd41_available) {
            float diff_bmp_scd = data.temperature_bmp280 - data.temperature_scd41;
            ESP_LOGI(TAG, "Temp diff (BMP280-SCD41): %.2f°C", diff_bmp_scd);
        }
        
        // DS3231과 다른 센서들 온도 차이 분석
        if (rtc_temp_valid) {
            ESP_LOGI(TAG, "DS3231 vs Average: %.2f°C (diff: %.2f°C)", rtc_temp, rtc_temp - avg_temp);
            
            if (data.aht20_available) {
                float diff_rtc_aht = rtc_temp - data.temperature_aht20;
                ESP_LOGI(TAG, "Temp diff (DS3231-AHT20): %.2f°C", diff_rtc_aht);
            }
        }
    } else if (rtc_temp_valid) {
        ESP_LOGI(TAG, "Only DS3231 temperature available: %.2f°C", rtc_temp);
    }
    
    // CO2 level analysis
    if (data.scd41_available) {
        if (data.co2_scd41 < 800) {
            ESP_LOGI(TAG, "CO2 level: Good (%.0fppm)", data.co2_scd41);
        } else if (data.co2_scd41 < 1000) {
            ESP_LOGW(TAG, "CO2 level: Moderate (%.0fppm)", data.co2_scd41);
        } else if (data.co2_scd41 < 1500) {
            ESP_LOGW(TAG, "CO2 level: Poor (%.0fppm)", data.co2_scd41);
        } else {
            ESP_LOGE(TAG, "CO2 level: Very poor (%.0fppm)", data.co2_scd41);
        }
    }

    // VOC level analysis
    if (data.sgp40_available) {
        if (data.voc_index_sgp40 < 100) {
            ESP_LOGI(TAG, "VOC Index: Excellent (%.0f)", data.voc_index_sgp40);
        } else if (data.voc_index_sgp40 < 200) {
            ESP_LOGI(TAG, "VOC Index: Good (%.0f)", data.voc_index_sgp40);
        } else if (data.voc_index_sgp40 < 300) {
            ESP_LOGW(TAG, "VOC Index: Moderate (%.0f)", data.voc_index_sgp40);
        } else {
            ESP_LOGW(TAG, "VOC Index: Poor (%.0f)", data.voc_index_sgp40);
        }
    }
}

// 프로토타입용 추가 기능들
void SensorManager::scanI2CBus() {
    ESP_LOGI(TAG, "🔍 I2C 버스 스캔 시작...");
    ESP_LOGI(TAG, "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");
    
    for (int i = 0; i < 128; i += 16) {
        printf("I (%s) %02x:", TAG, i);
        for (int j = 0; j < 16; j++) {
            fflush(stdout);
            uint8_t address = i + j;
            
            // 예약된 주소들은 건너뛰기
            if (address < 0x03 || address > 0x77) {
                printf(" --");
                continue;
            }
            
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
            i2c_master_stop(cmd);
            
            esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
            i2c_cmd_link_delete(cmd);
            
            if (ret == ESP_OK) {
                printf(" %02x", address);
            } else {
                printf(" --");
            }
        }
        printf("\n");
    }
    ESP_LOGI(TAG, "I2C 버스 스캔 완료");
}

bool SensorManager::checkSensorConnections() {
    ESP_LOGI(TAG, "🔌 센서 연결 상태 체크...");
    bool all_connected = true;
    
    // 각 센서 주소에 ping 테스트
    struct {
        uint8_t address;
        const char* name;
        bool* initialized;
    } sensors[] = {
        {0x38, "AHT20", &aht20_initialized},
        {0x76, "BMP280(0x76)", &bmp280_initialized},
        {0x77, "BMP280(0x77)", &bmp280_initialized}, 
        {0x62, "SCD41", &scd41_initialized},
        {0x59, "SGP40", &sgp40_initialized},
        {0x40, "INA226(0x40)", &ina226_initialized},
        {0x41, "INA226(0x41)", &ina226_initialized},
        {0x68, "DS3231", &rtc_initialized}
    };
    
    for (const auto& sensor : sensors) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (sensor.address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✅ %s (0x%02X): 연결됨", sensor.name, sensor.address);
        } else {
            if (*(sensor.initialized)) {
                ESP_LOGW(TAG, "⚠️  %s (0x%02X): 초기화됨이지만 응답 없음!", sensor.name, sensor.address);
                all_connected = false;
            } else {
                ESP_LOGD(TAG, "❌ %s (0x%02X): 미연결", sensor.name, sensor.address);
            }
        }
    }
    
    return all_connected;
}

bool SensorManager::validateSensorData(const SensorData &data) {
    bool all_valid = true;
    
    // AHT20 데이터 검증
    if (data.aht20_available) {
        if (data.temperature_aht20 < TEMP_MIN_C || data.temperature_aht20 > TEMP_MAX_C) {
            ESP_LOGW(TAG, "⚠️  AHT20 온도 이상값: %.2f°C (정상범위: %.1f~%.1f°C)", 
                     data.temperature_aht20, TEMP_MIN_C, TEMP_MAX_C);
            all_valid = false;
        }
        if (data.humidity_aht20 < HUMIDITY_MIN_PCT || data.humidity_aht20 > HUMIDITY_MAX_PCT) {
            ESP_LOGW(TAG, "⚠️  AHT20 습도 이상값: %.2f%% (정상범위: %.1f~%.1f%%)", 
                     data.humidity_aht20, HUMIDITY_MIN_PCT, HUMIDITY_MAX_PCT);
            all_valid = false;
        }
    }
    
    // BMP280 데이터 검증  
    if (data.bmp280_available) {
        if (data.temperature_bmp280 < TEMP_MIN_C || data.temperature_bmp280 > TEMP_MAX_C) {
            ESP_LOGW(TAG, "⚠️  BMP280 온도 이상값: %.2f°C (정상범위: %.1f~%.1f°C)", 
                     data.temperature_bmp280, TEMP_MIN_C, TEMP_MAX_C);
            all_valid = false;
        }
        if (data.pressure_bmp280 < PRESSURE_MIN_HPA || data.pressure_bmp280 > PRESSURE_MAX_HPA) {
            ESP_LOGW(TAG, "⚠️  BMP280 기압 이상값: %.2fhPa (정상범위: %.1f~%.1fhPa)", 
                     data.pressure_bmp280, PRESSURE_MIN_HPA, PRESSURE_MAX_HPA);
            all_valid = false;
        }
    }
    
    // SCD41 데이터 검증
    if (data.scd41_available) {
        if (data.temperature_scd41 < TEMP_MIN_C || data.temperature_scd41 > TEMP_MAX_C) {
            ESP_LOGW(TAG, "⚠️  SCD41 온도 이상값: %.2f°C (정상범위: %.1f~%.1f°C)", 
                     data.temperature_scd41, TEMP_MIN_C, TEMP_MAX_C);
            all_valid = false;
        }
        if (data.humidity_scd41 < HUMIDITY_MIN_PCT || data.humidity_scd41 > HUMIDITY_MAX_PCT) {
            ESP_LOGW(TAG, "⚠️  SCD41 습도 이상값: %.2f%% (정상범위: %.1f~%.1f%%)", 
                     data.humidity_scd41, HUMIDITY_MIN_PCT, HUMIDITY_MAX_PCT);
            all_valid = false;
        }
        if (data.co2_scd41 < CO2_MIN_PPM || data.co2_scd41 > CO2_MAX_PPM) {
            ESP_LOGW(TAG, "⚠️  SCD41 CO2 이상값: %.0fppm (정상범위: %.0f~%.0fppm)", 
                     data.co2_scd41, CO2_MIN_PPM, CO2_MAX_PPM);
            all_valid = false;
        }
    }
    
    // SGP40 데이터 검증
    if (data.sgp40_available) {
        if (data.voc_index_sgp40 < VOC_MIN_INDEX || data.voc_index_sgp40 > VOC_MAX_INDEX) {
            ESP_LOGW(TAG, "⚠️  SGP40 VOC Index 이상값: %.0f (정상범위: %.0f~%.0f)", 
                     data.voc_index_sgp40, VOC_MIN_INDEX, VOC_MAX_INDEX);
            all_valid = false;
        }
    }
    
    if (all_valid) {
        ESP_LOGD(TAG, "✅ 모든 센서 데이터가 정상 범위 내에 있습니다");
    }
    
    return all_valid;
}

bool SensorManager::recoverFailedSensors() {
    ESP_LOGI(TAG, "🔧 실패한 센서 복구 시도...");
    bool any_recovered = false;
    
    // I2C 버스 재초기화
    ESP_LOGD(TAG, "I2C 버스 재초기화...");
    i2c_driver_delete(I2C_MASTER_NUM);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_PIN;
    conf.scl_io_num = I2C_SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = 400000;
    conf.clk_flags = 0;
    
    if (i2c_param_config(I2C_MASTER_NUM, &conf) != ESP_OK ||
        i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0) != ESP_OK) {
        ESP_LOGE(TAG, "I2C 재초기화 실패!");
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // 각 센서 개별 복구 시도
    if (!aht20_initialized && aht20_sensor) {
        ESP_LOGI(TAG, "AHT20 복구 시도...");
        if (aht20_sensor->init()) {
            aht20_initialized = true;
            any_recovered = true;
            ESP_LOGI(TAG, "✅ AHT20 복구 성공!");
        } else {
            ESP_LOGW(TAG, "❌ AHT20 복구 실패");
        }
    }
    
    if (!bmp280_initialized && bmp280_sensor) {
        ESP_LOGI(TAG, "BMP280 복구 시도...");
        if (bmp280_sensor->init()) {
            bmp280_initialized = true;
            any_recovered = true;
            ESP_LOGI(TAG, "✅ BMP280 복구 성공!");
        } else {
            ESP_LOGW(TAG, "❌ BMP280 복구 실패");
        }
    }
    
    if (!scd41_initialized && scd41_sensor) {
        ESP_LOGI(TAG, "SCD41 복구 시도...");
        if (scd41_sensor->init()) {
            scd41_initialized = true;
            any_recovered = true;
            ESP_LOGI(TAG, "✅ SCD41 복구 성공!");
        } else {
            ESP_LOGW(TAG, "❌ SCD41 복구 실패");
        }
    }
    
    if (!sgp40_initialized && sgp40_sensor) {
        ESP_LOGI(TAG, "SGP40 복구 시도...");
        if (sgp40_sensor->init()) {
            sgp40_initialized = true;
            any_recovered = true;
            ESP_LOGI(TAG, "✅ SGP40 복구 성공!");
        } else {
            ESP_LOGW(TAG, "❌ SGP40 복구 실패");
        }
    }
    
    if (!ina226_initialized && ina226_monitor) {
        ESP_LOGI(TAG, "INA226 복구 시도...");
        if (ina226_monitor->init()) {
            ina226_initialized = true;
            any_recovered = true;
            ESP_LOGI(TAG, "✅ INA226 복구 성공!");
        } else {
            ESP_LOGW(TAG, "❌ INA226 복구 실패");
        }
    }
    
    if (!rtc_initialized && rtc_module) {
        ESP_LOGI(TAG, "DS3231 복구 시도...");
        if (rtc_module->init()) {
            rtc_initialized = true;
            any_recovered = true;
            ESP_LOGI(TAG, "✅ DS3231 복구 성공!");
        } else {
            ESP_LOGW(TAG, "❌ DS3231 복구 실패");
        }
    }
    
    if (any_recovered) {
        ESP_LOGI(TAG, "🎉 센서 복구 완료! 현재 작동 센서: %d개", getWorkingSensorCount());
    } else {
        ESP_LOGW(TAG, "센서 복구 시도 완료 - 복구된 센서 없음");
    }
    
    return any_recovered;
}

bool SensorManager::testPullupResistors() {
    ESP_LOGI(TAG, "🧪 풀업저항 테스트 시작...");
    
    // 현재 설정 (외부 풀업 가정)으로 스캔
    ESP_LOGI(TAG, "1️⃣ 외부 풀업저항 테스트 (내장 풀업 OFF)");
    scanI2CBus();
    
    // 센서 개수 확인
    int external_pullup_count = 0;
    for (int addr = 0x08; addr <= 0x77; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            external_pullup_count++;
        }
    }
    
    ESP_LOGI(TAG, "외부 풀업 테스트 결과: %d개 기기 감지", external_pullup_count);
    
    // I2C 드라이버 재설정 (내장 풀업 활성화)
    ESP_LOGI(TAG, "2️⃣ 내장 풀업저항 테스트 (내장 풀업 ON)");
    
    i2c_driver_delete(I2C_MASTER_NUM);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_PIN;
    conf.scl_io_num = I2C_SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;   // 내장 풀업 활성화
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;   // 내장 풀업 활성화
    conf.master.clk_speed = 100000; // 내장 풀업은 100kHz가 더 안정적
    conf.clk_flags = 0;
    
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    scanI2CBus();
    
    // 센서 개수 확인
    int internal_pullup_count = 0;
    for (int addr = 0x08; addr <= 0x77; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            internal_pullup_count++;
        }
    }
    
    ESP_LOGI(TAG, "내장 풀업 테스트 결과: %d개 기기 감지", internal_pullup_count);
    
    // 결과 분석 및 권장사항
    ESP_LOGI(TAG, "📊 풀업저항 테스트 결과 분석:");
    if (external_pullup_count > internal_pullup_count) {
        ESP_LOGI(TAG, "✅ 권장: 외부 풀업저항 사용 (400kHz 가능)");
        ESP_LOGI(TAG, "   → 센서 모듈에 풀업저항이 있습니다!");
        return true;  // 외부 풀업 사용 권장
    } else if (internal_pullup_count > external_pullup_count) {
        ESP_LOGI(TAG, "⚠️  권장: 내장 풀업저항 사용 (100kHz 권장)");
        ESP_LOGI(TAG, "   → 센서 모듈에 풀업저항이 없거나 부족합니다");
        return false; // 내장 풀업 사용 권장
    } else {
        ESP_LOGI(TAG, "🤔 결과 동일 - 외부 풀업 사용 (더 안정적)");
        return true;
    }
}

// 전력 관리 기능들 (기본 구조)
bool SensorManager::setPowerMode(power_mode_t mode) {
    if (mode == current_power_mode) {
        ESP_LOGD(TAG, "이미 요청된 전력 모드입니다: %d", mode);
        return true;
    }
    
    ESP_LOGI(TAG, "전력 모드 변경: %d -> %d", current_power_mode, mode);
    
    switch (mode) {
        case POWER_MODE_NORMAL:
            ESP_LOGI(TAG, "일반 모드로 전환 (HP Core I2C GPIO22/23)");
            current_power_mode = POWER_MODE_NORMAL;
            // 현재 이미 HP Core 모드로 구현되어 있음
            return true;
            
        case POWER_MODE_LOW_POWER:
            ESP_LOGW(TAG, "저전력 모드는 아직 구현되지 않았습니다 (LP Core I2C GPIO6/7)");
            ESP_LOGW(TAG, "향후 구현 예정: ulp-lp-core 기능 사용");
            return false;  // 아직 구현 안됨
            
        default:
            ESP_LOGE(TAG, "알 수 없는 전력 모드: %d", mode);
            return false;
    }
}

power_mode_t SensorManager::getCurrentPowerMode() const {
    return current_power_mode;
}

bool SensorManager::initLpCoreI2C() {
    ESP_LOGW(TAG, "LP Core I2C 초기화는 향후 구현 예정");
    ESP_LOGW(TAG, "필요한 기능:");
    ESP_LOGW(TAG, "  - ESP-IDF ulp-lp-core 컴포넌트 활용");
    ESP_LOGW(TAG, "  - GPIO6(SDA)/GPIO7(SCL) 핀 사용");
    ESP_LOGW(TAG, "  - HP Core에서 LP Core로 센서 제어 이관");
    ESP_LOGW(TAG, "  - Deep Sleep 모드에서 센서 모니터링");
    
    lp_core_i2c_initialized = false;
    return false;  // 아직 구현 안됨
}

// DS3231 RTC 기능 구현
bool SensorManager::initRTC() {
    ESP_LOGI(TAG, "DS3231 RTC 초기화 시작...");
    
    if (!rtc_module) {
        ESP_LOGE(TAG, "RTC 모듈이 생성되지 않음");
        return false;
    }
    
    if (rtc_module->init()) {
        rtc_initialized = true;
        ESP_LOGI(TAG, "✅ DS3231 RTC 초기화 성공");
        return true;
    } else {
        ESP_LOGW(TAG, "⚠️  DS3231 RTC 초기화 실패");
        rtc_initialized = false;
        return false;
    }
}

bool SensorManager::setSystemTime(uint16_t year, uint8_t month, uint8_t date, 
                                  uint8_t hours, uint8_t minutes, uint8_t seconds) {
    if (!rtc_initialized || !rtc_module) {
        ESP_LOGE(TAG, "RTC가 초기화되지 않음");
        return false;
    }
    
    rtc_time_t time;
    time.year = year;
    time.month = month;
    time.date = date;
    time.hours = hours;
    time.minutes = minutes;
    time.seconds = seconds;
    time.day_of_week = 1; // 기본값
    
    if (rtc_module->setTime(time)) {
        ESP_LOGI(TAG, "시스템 시간 설정: %04d-%02d-%02d %02d:%02d:%02d",
                 year, month, date, hours, minutes, seconds);
        return true;
    } else {
        ESP_LOGE(TAG, "시스템 시간 설정 실패");
        return false;
    }
}

bool SensorManager::getSystemTime() {
    if (!rtc_initialized || !rtc_module) {
        ESP_LOGW(TAG, "RTC가 초기화되지 않음 - 시간 읽기 불가");
        return false;
    }
    
    rtc_time_t time;
    if (rtc_module->getTime(time)) {
        ESP_LOGI(TAG, "현재 시간: %04d-%02d-%02d %02d:%02d:%02d (요일: %d)",
                 time.year, time.month, time.date, 
                 time.hours, time.minutes, time.seconds, time.day_of_week);
        return true;
    } else {
        ESP_LOGE(TAG, "시간 읽기 실패");
        return false;
    }
}

float SensorManager::getRTCTemperature() {
    if (!rtc_initialized || !rtc_module) {
        ESP_LOGW(TAG, "RTC가 초기화되지 않음 - 온도 읽기 불가");
        return -999.0f;
    }
    
    float temp = rtc_module->getTemperature();
    if (temp > -50.0f && temp < 100.0f) {
        ESP_LOGD(TAG, "DS3231 온도: %.2f°C", temp);
    } else {
        ESP_LOGW(TAG, "DS3231 온도 읽기 실패: %.2f°C", temp);
    }
    
    return temp;
}

bool SensorManager::setAdaptivePowerMode(uint8_t mode) {
    if (!rtc_initialized || !rtc_module) {
        ESP_LOGW(TAG, "RTC가 초기화되지 않음 - 절전 모드 설정 불가");
        return false;
    }
    
    adaptive_power_mode_t power_mode;
    switch (mode) {
        case 0:
            power_mode = ADAPTIVE_POWER_NORMAL;
            break;
        case 1:
            power_mode = ADAPTIVE_POWER_SAVE;
            break;
        case 2:
            power_mode = ADAPTIVE_POWER_EMERGENCY;
            break;
        default:
            ESP_LOGE(TAG, "잘못된 절전 모드: %d", mode);
            return false;
    }
    
    if (rtc_module->setAdaptivePowerMode(power_mode)) {
        ESP_LOGI(TAG, "적응형 절전 모드 설정: %d", mode);
        return true;
    } else {
        ESP_LOGE(TAG, "적응형 절전 모드 설정 실패");
        return false;
    }
}

bool SensorManager::checkBroadcastMessages() {
    if (!rtc_initialized || !rtc_module) {
        return false;
    }
    
    broadcast_message_t message;
    if (rtc_module->checkBroadcastMessage(&message)) {
        ESP_LOGI(TAG, "브로드캐스트 메시지 수신: 타입=%d, 레벨=%d",
                 message.message_type, message.alert_level);
        
        // 모드 변경 메시지인 경우 자동으로 절전 모드 적용
        if (message.message_type == MSG_MODE_CHANGE) {
            return setAdaptivePowerMode(message.alert_level);
        }
        
        return true;
    }
    
    return false;
}

bool SensorManager::isWakeupByAlarm() {
    if (!rtc_initialized || !rtc_module) {
        return false;
    }
    
    bool alarm_triggered = rtc_module->isAlarm1Triggered();
    if (alarm_triggered) {
        ESP_LOGI(TAG, "RTC 알람에 의한 Wake-up 감지");
        rtc_module->clearAlarm1(); // 알람 플래그 클리어
    }
    
    return alarm_triggered;
}