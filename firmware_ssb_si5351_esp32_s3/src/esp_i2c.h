#ifndef ESP_I2C_H_
#define ESP_I2C_H_

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t polar_i2c_init(uint8_t address, uint32_t speed_hz);
uint32_t polar_i2c_default_speed_hz(void);
esp_err_t polar_i2c_set_speed(uint32_t speed_hz);
esp_err_t polar_i2c_enable_async(void);
esp_err_t polar_i2c_write(const uint8_t *data, size_t length);
esp_err_t polar_i2c_submit(const uint8_t *data, size_t length);
esp_err_t polar_i2c_write_read(const uint8_t *write_data, size_t write_length,
                               uint8_t *read_data, size_t read_length);
esp_err_t polar_i2c_wait(void);

#endif
