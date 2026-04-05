#!/bin/bash
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

RATE=${2:-16000}
SPEECH_DSP=${SPEECH_DSP:-1}

echo "Building simulator (SAMPLE_RATE=$RATE, USE_SPEECH_DSP=$SPEECH_DSP)..."

g++ -O2 -DSAMPLE_RATE=$RATE -DUSE_SPEECH_DSP=$SPEECH_DSP \
    -I../src \
    simulate_loopback.cpp \
    ../src/polar_mod.cpp \
    ../src/speech_dsp.cpp \
    -lm -o simulate_loopback

if [ $? -ne 0 ]; then
    echo "Build failed!"
    exit 1
fi

echo ""
./simulate_loopback "$1"
