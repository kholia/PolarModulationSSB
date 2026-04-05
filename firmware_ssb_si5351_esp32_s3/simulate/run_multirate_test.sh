#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd "$(dirname "$0")" && pwd)
root_dir=$(cd "$script_dir/.." && pwd)
binary=$(mktemp "${TMPDIR:-/tmp}/polar-multirate-test.XXXXXX")
trap 'rm -f "$binary"' EXIT

${CXX:-g++} -std=c++17 -O2 -Wall -Wextra -Werror \
    -DUSE_SPEECH_DSP=1 -I"$root_dir/src" \
    "$script_dir/test_multirate.cpp" \
    "$root_dir/src/polar_mod.cpp" \
    "$root_dir/src/tx_dsp.cpp" \
    "$root_dir/src/speech_dsp.cpp" \
    -lm -o "$binary"

"$binary"
