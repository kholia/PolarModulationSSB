cmake_minimum_required(VERSION 3.12)

# Accept serial number as build parameter
# Usage: cmake -DAUTHORIZED_SERIAL=E66160D0E77B5A28 ..
if(DEFINED AUTHORIZED_SERIAL)
    message(STATUS "Building firmware for authorized serial: ${AUTHORIZED_SERIAL}")
    add_compile_definitions(AUTHORIZED_SERIAL="${AUTHORIZED_SERIAL}")
else()
    message(STATUS "Building DEMO firmware (no serial restriction)")
endif()

# rest of your project
# Full SSB firmware with polar modulation
add_executable(usb_analog_microphone
    main.cpp
    ad9850.cpp
    polar_mod.cpp
    arduino_compat.cpp
    Adafruit_NeoPixel.cpp
)

# Simple carrier test (commented out for SSB):
# add_executable(usb_analog_microphone
#     main_simple.cpp
#     ad9850.cpp
# )

pico_generate_pio_header(usb_analog_microphone ${CMAKE_CURRENT_LIST_DIR}/ws2812byte.pio)

# Sample rate: 32000 (now feasible with 10 MHz SPI to AD9850)
set(SAMPLE_RATE 32000 CACHE STRING "Audio sample rate in Hz")
# Audio mode: HIFI (full bandwidth, 257-tap Hilbert) or SSB_NARROW (300-3000 Hz, 127-tap Hilbert)
set(AUDIO_MODE "HIFI" CACHE STRING "Audio mode: HIFI or SSB_NARROW")

# Set compile definitions - HIFI mode only
set(MODE_DEFS SAMPLE_RATE=${SAMPLE_RATE} USE_HIFI=1)
message(STATUS "Audio mode: HIFI (full bandwidth, 257-tap Hilbert)")
target_compile_definitions(usb_analog_microphone PRIVATE ${MODE_DEFS})

# PIO I2C: conditionally compile pio_i2c.c and generate PIO header
# (Skip PIO I2C since AD9850 uses SPI)
set(USE_PIO_I2C 0 CACHE STRING "Use PIO-based I2C (only needed for Si4735 receiver)")
if(USE_PIO_I2C)
    target_compile_definitions(usb_analog_microphone PRIVATE USE_PIO_I2C=1)
    target_sources(usb_analog_microphone PRIVATE pio_i2c.c)
    pico_generate_pio_header(usb_analog_microphone ${CMAKE_CURRENT_LIST_DIR}/i2c.pio)
endif()

target_include_directories(usb_analog_microphone PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# Note: AD9850 uses SPI, but NeoPixel needs PIO, and main.cpp still references I2C
target_link_libraries(usb_analog_microphone PRIVATE pico_analog_microphone hardware_adc hardware_dma hardware_pwm hardware_watchdog hardware_spi hardware_pio hardware_i2c)

# enable usb output, disable uart output
pico_enable_stdio_usb(usb_analog_microphone 1)
pico_enable_stdio_uart(usb_analog_microphone 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(usb_analog_microphone)
