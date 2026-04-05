#ifndef DDX_common
#define DDX_common

#define VERSION "27.0"
#define BUILD 1

#include <stdio.h>
#include <stdint.h>

#include "ad9850.h"

#include "hardware/watchdog.h"
// #include <EEPROM.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include "hardware/pwm.h"
#include "pico/multicore.h"
#include "hardware/adc.h"
#include "hardware/uart.h"

// Pinout
#define PTT 14  // PTT pin
#define RST 5   // Reset pin

// AD9850 SPI pins (using spi0)
// NOTE: CS and FQ_UD share GPIO 1 (same as working AD9850_demo)
#define AD9850_SPI_PORT spi0
#define AD9850_CS 1               // SPI chip select (GP1) - also FQ_UD
#define AD9850_SPI_SCK 2          // SPI clock (GP2)
#define AD9850_SPI_MOSI 3         // SPI data (GP3)
#define AD9850_RESET 8            // Reset pin (GP8)
#define AD9850_FQ_UD 1            // Frequency update (shared with CS!)
#define AD9850_SPI_BAUD 10000000  // 10 MHz SPI clock

// I2C1 pinout - DDX-Commercial-27
#define I2C1_SDA 6
#define I2C1_SCL 7

extern uint64_t freq;

// I2C1 initialization (used by Si4735 receiver if present)
void init_i2c1(void);

#ifdef USE_PIO_I2C
#include "hardware/pio.h"
extern PIO i2c_pio_inst;
extern uint i2c_pio_sm;
#define PIO_I2C_PIN_HS 0xFF  // No external HS switching circuitry
#endif

void serialEvent();
void led_flash();

#endif
