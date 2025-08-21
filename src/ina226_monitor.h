#ifndef INA226_MONITOR_H
#define INA226_MONITOR_H

#include "sensor_types.h"
#include <stdint.h>

class INA226_Monitor {
public:
    INA226_Monitor(float shunt_resistance_ohm, float max_expected_current_a);

    bool init();
    bool readStatus(BatteryStatus *status);
    uint8_t getAddress() const { return detected_addr_; }

private:
    bool initialized_;
    uint8_t detected_addr_;
    float shunt_resistance_;
    float max_current_;
    float current_lsb_;
    uint16_t cal_value_;

    bool detectSensor();
    bool configure();
    bool calibrate();
    bool writeRegister(uint8_t reg, uint16_t value);
    bool readRegister(uint8_t reg, uint16_t *value);
};

#endif // INA226_MONITOR_H