idf_component_register(
    SRCS
        "main.cpp"
        "si5351.cpp"
        "polar_mod.cpp"
        "tx_dsp.cpp"
        "speech_dsp.cpp"
        "esp_i2c.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_ledc esp_hal_i2c esp_timer
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Wno-unused-function)
target_compile_definitions(${COMPONENT_LIB} PRIVATE USE_SPEECH_DSP=1)
