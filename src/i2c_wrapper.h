#ifndef I2C_WRAPPER_H
#define I2C_WRAPPER_H

#include "driver/i2c.h"

namespace I2CWrapper {

esp_err_t write_bytes(i2c_port_t port, uint8_t device_addr, const uint8_t* data, size_t len, uint32_t timeout_ms);
esp_err_t read_bytes(i2c_port_t port, uint8_t device_addr, uint8_t* data, size_t len, uint32_t timeout_ms);
esp_err_t write_then_read(i2c_port_t port, uint8_t device_addr, const uint8_t* write_buf, size_t write_len, uint8_t* read_buf, size_t read_len, uint32_t timeout_ms);

} // namespace I2CWrapper

#endif // I2C_WRAPPER_H