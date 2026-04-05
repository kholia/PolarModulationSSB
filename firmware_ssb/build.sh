#!/usr/bin/env bash

export PICO_BOARD=waveshare_rp2350_zero

cmake .

make -j8
