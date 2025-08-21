#include "i2c_wrapper.h"
#include "freertos/FreeRTOS.h"

namespace I2CWrapper {

esp_err_t write_bytes(i2c_port_t port, uint8_t device_addr, const uint8_t* data, size_t len, uint32_t timeout_ms) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    if (len > 0) {
        i2c_master_write(cmd, data, len, true);
    }
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(timeout_ms));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t read_bytes(i2c_port_t port, uint8_t device_addr, uint8_t* data, size_t len, uint32_t timeout_ms) {
    if (len == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(timeout_ms));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t write_then_read(i2c_port_t port, uint8_t device_addr, const uint8_t* write_buf, size_t write_len, uint8_t* read_buf, size_t read_len, uint32_t timeout_ms) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, write_buf, write_len, true);
    i2c_master_start(cmd); // Repeated start
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, read_buf, read_len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(timeout_ms));
    i2c_cmd_link_delete(cmd);
    return ret;
}

} // namespace I2CWrapper