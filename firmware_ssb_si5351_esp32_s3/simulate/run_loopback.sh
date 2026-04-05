#!/usr/bin/env bash
# ── Full SSB Loopback Simulator ──────────────────────────────────────────────
#
# Usage:
#   ./run_loopback.sh                       # uses embedded audio_data.h
#   ./run_loopback.sh input.wav             # reads any WAV file
#   ./run_loopback.sh input.wav 32000       # override sample rate
#
# Outputs:
#   loopback_out.wav    — full loopback (speech DSP + polar mod + demod)
#   speech_dsp_out.wav  — speech DSP only (no modulation, for comparison)

set -euo pipefail

script_dir=$(cd "$(dirname "$0")" && pwd)
root_dir=$(cd "$script_dir/.." && pwd)
input=${1:-}
RATE=${2:-16000}
SPEECH_DSP=${SPEECH_DSP:-1}
SIMULATE_DRAIN_AM=${SIMULATE_DRAIN_AM:-0}
SIMULATE_QUANTIZATION=${SIMULATE_QUANTIZATION:-1}
binary=$(mktemp "${TMPDIR:-/tmp}/polar-loopback.XXXXXX")
trap 'rm -f "$binary"' EXIT

echo "Building simulator (SAMPLE_RATE=$RATE, USE_SPEECH_DSP=$SPEECH_DSP, SIMULATE_DRAIN_AM=$SIMULATE_DRAIN_AM)..."

${CXX:-g++} -O2 -DSAMPLE_RATE="$RATE" -DUSE_SPEECH_DSP="$SPEECH_DSP" \
    -DSIMULATE_DRAIN_AM="$SIMULATE_DRAIN_AM" \
    -DSIMULATE_QUANTIZATION="$SIMULATE_QUANTIZATION" \
    -I"$root_dir/src" \
    "$script_dir/simulate_loopback.cpp" \
    "$root_dir/src/polar_mod.cpp" \
    "$root_dir/src/tx_dsp.cpp" \
    "$root_dir/src/speech_dsp.cpp" \
    -lm -o "$binary"

echo ""
if [ -n "$input" ]; then
    "$binary" "$input"
else
    "$binary"
fi
