cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/src")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(polar_ssb_si5351_esp32s3)
