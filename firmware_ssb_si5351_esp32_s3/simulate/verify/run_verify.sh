#!/usr/bin/env bash
# Verification suite for the ESP32-S3 polar SSB firmware. See README.md.
#   simulate/verify/run_verify.sh [all|build|dsp|timing|drain|twotone|c5bench]
set -euo pipefail

verify_dir=$(cd "$(dirname "$0")" && pwd)
root_dir=$(cd "$verify_dir/../.." && pwd)
out_dir=${OUT_DIR:-$verify_dir/out}
idf_image=${IDF_IMAGE:-espressif/idf:latest}
step=${1:-all}
mkdir -p "$out_dir"
py() { uv run -q --with numpy python "$@"; }

build() {
    echo "== Firmware build (separate build-verify/ dir, -Wall -Wextra)"
    docker run --rm -v "$root_dir:/project" -w /project "$idf_image" \
        bash -lc 'idf.py -B build-verify -D SDKCONFIG=build-verify/sdkconfig build' \
        > "$out_dir/build.log" 2>&1
    echo "warnings: $(grep -c 'warning:' "$out_dir/build.log" || true)"
    ls -l "$root_dir/build-verify/polar_ssb_si5351_esp32s3.bin"
}

dsp() {
    echo "== Host DSP tests"
    "$root_dir/simulate/run_multirate_test.sh"
    (cd "$root_dir/simulate" && ./run_loopback.sh "" 32000 | tail -8)
    ${CXX:-g++} -std=c++17 -O2 -Wall -Wextra -Werror -I"$root_dir/src" \
        "$root_dir/simulate/test_drain_am.cpp" -o "$out_dir/test_drain_am"
    "$out_dir/test_drain_am"
}

dump_tool() {
    ${CXX:-g++} -std=c++17 -O2 -DUSE_SPEECH_DSP=1 -I"$root_dir/src" \
        "$verify_dir/dump_dsp_stream.cpp" "$root_dir/src/tx_dsp.cpp" \
        "$root_dir/src/polar_mod.cpp" "$root_dir/src/speech_dsp.cpp" \
        -o "$out_dir/dump_dsp_stream"
}

timing() {
    dump_tool
    cd "$verify_dir"
    echo "== Si5351 write lengths per band (10 s speech, 32 ksample/s)"
    "$out_dir/dump_dsp_stream" 32000 wav 10 > "$out_dir/wav32.txt"
    for f in 3700000 7100000 14200000 21200000 28400000; do
        py timing_model.py "$out_dir/wav32.txt" "$f"
    done
    echo "== Two-tone (0.2 + 0.2, below compressor) vs update timing, default DSP"
    "$out_dir/dump_dsp_stream" 32000 two 2 0.2 > "$out_dir/two32.txt"
    py spec_model.py "$out_dir/two32.txt"
    echo "== Same, phase-step limiter disabled (PM_LIMITER=0)"
    PM_LIMITER=0 "$out_dir/dump_dsp_stream" 32000 two 2 0.2 > "$out_dir/two32_nolim.txt"
    CASES=A py spec_model.py "$out_dir/two32_nolim.txt"
    echo "== Amplitude/phase alignment sweep (negative = amplitude early)"
    FIXED_SHIFTS="18,0,-18,-31,-49" CASES=X py spec_model.py "$out_dir/two32_nolim.txt"
    echo "== 16 ksample/s, limiter disabled"
    PM_LIMITER=0 "$out_dir/dump_dsp_stream" 16000 two 2 0.2 > "$out_dir/two16_nolim.txt"
    RATE=16000 CASES=A py spec_model.py "$out_dir/two16_nolim.txt"
}

drain() {
    dump_tool
    cd "$verify_dir"
    echo "== Drain-AM firmware model: LEDC ticks, hardware fades, delay sweep"
    PM_LIMITER=0 "$out_dir/dump_dsp_stream" 32000 two 2 0.2 > "$out_dir/two32_nolim.txt"
    py firmware_am_model.py "$out_dir/two32_nolim.txt"
    echo "== Same at D=6,8 if LEDC fractional duty bits are not dithered"
    INT_DUTY=1 py firmware_am_model.py "$out_dir/two32_nolim.txt" 6,8
    echo "== 16 ksample/s profile"
    PM_LIMITER=0 "$out_dir/dump_dsp_stream" 16000 two 2 0.2 > "$out_dir/two16_nolim.txt"
    RATE=16000 py firmware_am_model.py "$out_dir/two16_nolim.txt" 4,8,12,16
}

twotone() {
    echo "== Two-tone generator float32 time-base error (old unwrapped form)"
    py "$verify_dir/twotone_precision.py"
}

c5bench() {
    echo "== Float DSP cost on a no-FPU RISC-V core (ESP32-C3 QEMU as C5 proxy)"
    docker run --rm -v "$root_dir:/project" -w /project/simulate/verify/c5_softfloat_bench \
        "$idf_image" bash -lc '
            idf.py set-target esp32c3 >/dev/null && idf.py build >/dev/null &&
            cd build && python -m esptool --chip esp32c3 merge-bin --pad-to-size 4MB \
                -o flash.bin @flash_args >/dev/null &&
            timeout 90 qemu-system-riscv32 -nographic -icount 3 -machine esp32c3 \
                -drive file=flash.bin,if=mtd,format=raw 2>&1 | grep -m4 -E "CAL|RATE"' |
        grep -E "CAL|RATE"
    echo "(counter = 1.28 x instructions with -icount 3; see CAL line)"
}

case $step in
    all) build; dsp; timing; drain; twotone; c5bench ;;
    build|dsp|timing|drain|twotone|c5bench) "$step" ;;
    *) echo "unknown step: $step" >&2; exit 2 ;;
esac
