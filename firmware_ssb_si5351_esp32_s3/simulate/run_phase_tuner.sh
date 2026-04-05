#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd "$(dirname "$0")" && pwd)
root_dir=$(cd "$script_dir/.." && pwd)
mode=${1:---one-factor}
output=${2:-$script_dir/phase_tuning.csv}
binary=$(mktemp "${TMPDIR:-/tmp}/polar-phase-tuner.XXXXXX")
trap 'rm -f "$binary"' EXIT

case "$mode" in
    --one-factor) tuner_arguments=() ;;
    --joint) tuner_arguments=(--joint) ;;
    --cleanup) tuner_arguments=(--cleanup) ;;
    *)
        echo "Usage: $0 [--one-factor|--joint|--cleanup] [output.csv]" >&2
        exit 2
        ;;
esac

${CXX:-g++} \
    -O3 \
    -std=c++17 \
    -DUSE_SPEECH_DSP=1 \
    -I"$root_dir/src" \
    "$script_dir/tune_phase_only.cpp" \
    "$root_dir/src/polar_mod.cpp" \
    "$root_dir/src/speech_dsp.cpp" \
    -lm \
    -o "$binary"

"$binary" "${tuner_arguments[@]}" | tee "$output"
echo "Sweep written to $output" >&2
