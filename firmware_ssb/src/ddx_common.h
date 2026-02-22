#ifndef DDX_common
#define DDX_common

#define VERSION "27.0"
#define BUILD 1

#include <stdio.h>
#include <stdint.h>

#include "si5351.h"

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

// I2C1 pinout - DDX-Commercial-27
#define I2C1_SDA 6
#define I2C1_SCL 7

extern uint64_t freq;

// I2C1 initialization (shared between Si5351 and Si4735)
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
