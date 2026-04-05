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
add_executable(usb_analog_microphone
    main.cpp
    si5351.cpp
    polar_mod.cpp
    speech_dsp.cpp
    arduino_compat.cpp
    Adafruit_NeoPixel.cpp
)

pico_generate_pio_header(usb_analog_microphone ${CMAKE_CURRENT_LIST_DIR}/ws2812byte.pio)

# Sample rate: 16000 or 32000 (32kHz needs I2C >= 2.0MHz with 2k pull-ups)
set(SAMPLE_RATE 16000 CACHE STRING "Audio sample rate in Hz")
# Speech DSP: 1 = enable AGC+clipper+filter pipeline, 0 = bypass
set(USE_SPEECH_DSP 1 CACHE STRING "Enable speech DSP processing")
# PIO I2C: 0 = hardware I2C (default), 1 = PIO-based I2C-HS (3.4MHz via pio-i2c-hs)
set(USE_PIO_I2C 1 CACHE STRING "Use PIO-based I2C instead of hardware I2C")
target_compile_definitions(usb_analog_microphone PRIVATE SAMPLE_RATE=${SAMPLE_RATE} USE_SPEECH_DSP=${USE_SPEECH_DSP})

# PIO I2C: conditionally compile pio_i2c.c and generate PIO header
if(USE_PIO_I2C)
    target_compile_definitions(usb_analog_microphone PRIVATE USE_PIO_I2C=1)
    target_sources(usb_analog_microphone PRIVATE pio_i2c.c)
    pico_generate_pio_header(usb_analog_microphone ${CMAKE_CURRENT_LIST_DIR}/i2c.pio)
endif()

target_include_directories(usb_analog_microphone PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(usb_analog_microphone PRIVATE pico_analog_microphone hardware_pio hardware_adc hardware_dma hardware_i2c hardware_pwm hardware_watchdog hardware_spi)

# enable usb output, disable uart output
pico_enable_stdio_usb(usb_analog_microphone 1)
pico_enable_stdio_uart(usb_analog_microphone 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(usb_analog_microphone)
