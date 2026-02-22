#!/bin/bash
g++ -O2 -I../src -I../ft8_lib/fft simulate_ssb.cpp ../src/polar_mod.cpp ../ft8_lib/fft/kiss_fft.c -lm -o simulate_ssb
if [ $? -eq 0 ]; then
    ./simulate_ssb > output.csv
    echo "Simulation finished. Results saved to output.csv"
    sort -t, -k2 -nr output.csv | head -n 10
fi
