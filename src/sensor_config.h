#ifndef SENSOR_CONFIG_H
#define SENSOR_CONFIG_H

#include <stdint.h>
#include "driver/gpio.h"

// I2C 핀 설정 - Xiao ESP32C6 올바른 I2C 핀
#define I2C_SDA_PIN GPIO_NUM_22  // D4
#define I2C_SCL_PIN GPIO_NUM_23  // D5

// AHT20 설정
#define AHT20_ADDR 0x38

// BMP280 설정  
#define BMP280_ADDR_0 0x76
#define BMP280_ADDR_1 0x77

// SCD41 설정
#define SCD41_ADDR 0x62

// 측정 간격
#define MEASUREMENT_INTERVAL_MS 5000

#endif