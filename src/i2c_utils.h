#ifndef I2C_UTILS_H
#define I2C_UTILS_H

#include <Arduino.h>
#include <Wire.h>

class I2CUtils {
public:
    // I2C 초기화
    static bool init();
    
    // I2C 스캔
    static void scanDevices();
    
    // I2C 쓰기/읽기 함수들
    static bool writeByte(uint8_t device_addr, uint8_t reg_addr, uint8_t data);
    static bool writeBytes(uint8_t device_addr, uint8_t *data, size_t len);
    static bool readBytes(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len);
    static bool readOnly(uint8_t device_addr, uint8_t *data, size_t len);
    
    // 디바이스 존재 확인
    static bool deviceExists(uint8_t device_addr);
    
private:
    static void printDeviceInfo(uint8_t address);
};

#endif // I2C_UTILS_H
