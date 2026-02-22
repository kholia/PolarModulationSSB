// Arduino compatibility layer for Pico SDK
// Maps Arduino functions to Pico SDK equivalents

#ifndef ARDUINO_COMPAT_H
#define ARDUINO_COMPAT_H

#include "stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

// Arduino data types
typedef unsigned char byte;
typedef bool boolean;

// Arduino constants
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2

// Time functions
#define delay(ms) sleep_ms(ms)
#define delayMicroseconds(us) sleep_us(us)
#define millis() to_ms_since_boot(get_absolute_time())

// GPIO functions
#define pinMode(pin, mode) \
  do { \
    gpio_init(pin); \
    if (mode == OUTPUT) { \
      gpio_set_dir(pin, GPIO_OUT); \
    } else if (mode == INPUT) { \
      gpio_set_dir(pin, GPIO_IN); \
      if (mode == INPUT_PULLUP) gpio_pull_up(pin); \
    } \
  } while (0)

#define digitalWrite(pin, value) gpio_put(pin, value)
#define digitalRead(pin) gpio_get(pin)

// I2C compatibility class
class TwoWire {
private:
  i2c_inst_t *i2c_instance;
  uint8_t address;
  uint8_t tx_buffer[32];  // Buffer for transaction data (Arduino Wire uses 32 bytes)
  size_t tx_buffer_index;
  uint8_t sda_pin;
  uint8_t scl_pin;

public:
  TwoWire(i2c_inst_t *i2c) : i2c_instance(i2c), address(0), tx_buffer_index(0), sda_pin(0), scl_pin(0) {}

  void setSDA(uint8_t pin) {
    sda_pin = pin;
  }

  void setSCL(uint8_t pin) {
    scl_pin = pin;
  }

  void begin() {
    if (sda_pin != 0 && scl_pin != 0) {
      i2c_init(i2c_instance, 100 * 1000);
      gpio_set_function(sda_pin, GPIO_FUNC_I2C);
      gpio_set_function(scl_pin, GPIO_FUNC_I2C);
      gpio_pull_up(sda_pin);
      gpio_pull_up(scl_pin);
    }
    // I2C should already be initialized
  }

  void beginTransmission(uint8_t addr) {
    address = addr;
    tx_buffer_index = 0;
  }

  uint8_t endTransmission() {
    if (tx_buffer_index > 0) {
      int result = i2c_write_blocking(i2c_instance, address, tx_buffer, tx_buffer_index, false);
      if (result == PICO_ERROR_GENERIC) {
        return 1;  // Error
      }
    }
    return 0;  // Success
  }

  size_t write(uint8_t data) {
    if (tx_buffer_index < sizeof(tx_buffer)) {
      tx_buffer[tx_buffer_index++] = data;
      return 1;
    }
    return 0;  // Buffer full
  }

  size_t write(const uint8_t *data, size_t len) {
    size_t written = 0;
    for (size_t i = 0; i < len && tx_buffer_index < sizeof(tx_buffer); i++) {
      tx_buffer[tx_buffer_index++] = data[i];
      written++;
    }
    return written;
  }

  size_t requestFrom(uint8_t addr, size_t len) {
    address = addr;
    // Note: This is a simplified implementation
    // Real Arduino Wire.requestFrom is more complex
    return len;
  }

  int read() {
    uint8_t data;
    i2c_read_blocking(i2c_instance, address, &data, 1, false);
    return data;
  }

  void setClock(uint32_t freq) {
    // For Pico SDK, I2C frequency is set during initialization
    // This is a no-op for compatibility
    (void)freq;
  }
};

// Global Wire instance
extern TwoWire Wire;
extern TwoWire Wire1;

// AVR program memory functions (for compatibility)
// On Pico, we don't have separate program memory, so just return the data directly
#define pgm_read_byte_near(address) (*((uint8_t *)(address)))
#define pgm_read_word_near(address) (*((uint16_t *)(address)))

#endif  // ARDUINO_COMPAT_H
