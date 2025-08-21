#include "sensor_manager.h"
#include "sensor_config.h"
#include "aht20_sensor.h"
#include "bmp280_sensor.h"
#include "scd41_sensor.h"
#include "sgp40_sensor.h"
#include "ina226_monitor.h"
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
#define INA226_SHUNT_RESISTANCE  0.1f 
// 예상되는 최대 전류 (Ampere). 이 값을 기준으로 측정 정밀도가 결정됩니다.
#define INA226_MAX_CURRENT       1.0f 

SensorManager::SensorManager() : aht20_sensor(nullptr), bmp280_sensor(nullptr), scd41_sensor(nullptr), 
                                 aht20_initialized(false), bmp280_initialized(false), scd41_initialized(false), sgp40_initialized(false), ina226_initialized(false) 
{
    aht20_sensor = std::make_unique<AHT20Sensor>();
    bmp280_sensor = std::make_unique<BMP280Sensor>();
    scd41_sensor = std::make_unique<SCD41Sensor>();
    sgp40_sensor = std::make_unique<SGP40Sensor>();
    ina226_monitor = std::make_unique<INA226_Monitor>(INA226_SHUNT_RESISTANCE, INA226_MAX_CURRENT);
}

SensorManager::~SensorManager() = default; // 스마트 포인터가 자동으로 메모리를 해제합니다.

bool SensorManager::init() {
    ESP_LOGI(TAG, "SensorManager initializing...");
    ESP_LOGI(TAG, "I2C pins: SDA=GPIO%d, SCL=GPIO%d", I2C_SDA_PIN, I2C_SCL_PIN);
    
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_PIN;
    conf.scl_io_num = I2C_SCL_PIN;
    // 400kHz Fast Mode 사용 시, 안정적인 신호 품질을 위해 외부 풀업 저항(예: 4.7kΩ) 사용을 권장합니다.
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = 400000; // 100kHz -> 400kHz (Fast Mode)로 속도 향상
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
    
    // Temperature comparison analysis
    int temp_sensors = 0;
    float temp_total = 0;
    
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