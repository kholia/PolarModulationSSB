# Benchmarks the firmware's float DSP on a RISC-V core without an FPU.
set(dsp "${CMAKE_CURRENT_LIST_DIR}/../../../../src")
idf_component_register(
    SRCS "bench.cpp" "${dsp}/tx_dsp.cpp" "${dsp}/speech_dsp.cpp" "${dsp}/polar_mod.cpp"
    INCLUDE_DIRS "." "${dsp}"
    REQUIRES esp_timer)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-function)
target_compile_definitions(${COMPONENT_LIB} PRIVATE USE_SPEECH_DSP=1)
