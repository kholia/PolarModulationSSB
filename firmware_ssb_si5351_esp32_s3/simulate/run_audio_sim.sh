#!/bin/bash
# Compile
g++ -O2 -I../src simulate_audio.cpp ../src/polar_mod.cpp -lm -o simulate_audio

if [ $? -eq 0 ]; then
    # Run
    ./simulate_audio
    
    # Convert to WAV (32kHz Mono)
    ffmpeg -y -f s16le -ar 32000 -ac 1 -i processed.raw output_sim.wav
    echo "Simulation finished. Listen to output_sim.wav"
fi
