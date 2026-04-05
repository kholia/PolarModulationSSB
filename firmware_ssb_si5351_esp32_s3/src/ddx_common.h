#ifndef DDX_COMMON_H_
#define DDX_COMMON_H_

#include <stdint.h>

#include "sdkconfig.h"

#define VERSION "28.0-esp32s3"
#define BUILD 1

#define PTT CONFIG_POLAR_PTT_GPIO
#define I2C1_SDA CONFIG_POLAR_I2C_SDA_GPIO
#define I2C1_SCL CONFIG_POLAR_I2C_SCL_GPIO

extern uint64_t freq;

#endif
