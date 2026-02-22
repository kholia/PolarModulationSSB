// Arduino compatibility layer for Pico SDK
#include "arduino_compat.h"

// Global Wire instance using i2c0 (for compatibility)
TwoWire Wire(i2c0);

// Global Wire1 instance using i2c1 (for DDX-Commercial-27 pinout)
TwoWire Wire1(i2c1);
