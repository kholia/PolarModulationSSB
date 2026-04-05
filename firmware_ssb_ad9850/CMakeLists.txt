cmake_minimum_required(VERSION 3.12)

# initialize pico_sdk from GIT
# (note this can come from environment, CMake cache etc)
# set(PICO_SDK_FETCH_FROM_GIT on)

# pico_sdk_import.cmake is a single file copied from this SDK
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(pico_microphone)

# initialize the Pico SDK
pico_sdk_init()

add_library(pico_analog_microphone INTERFACE)


target_include_directories(pico_analog_microphone INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/include
)

set(PICO_FLASH_SPI_CLKDIV 4)

add_subdirectory("src")
target_link_libraries(pico_analog_microphone INTERFACE pico_stdlib pico_multicore pico_runtime pico_runtime_init pico_sync hardware_adc hardware_dma hardware_spi pico_unique_id)
