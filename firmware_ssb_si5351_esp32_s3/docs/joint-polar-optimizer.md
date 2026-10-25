# Joint envelope/phase optimizer

`simulate/optimize_polar_joint.py` puts phase-only, classic polar (EER) and a
jointly optimized drain-envelope-plus-phase trajectory through the same
metrics and the same hardware model, so they can be compared directly.

## Run it

```sh
uv run simulate/optimize_polar_joint.py ESP32-S3-Si5351-Sample.wav \
  --mode joint --envelope-bandwidth 2000 --iterations 1000 \
  --phase-rate 0.05 --amplitude-rate 0.005 --stop-weight 3 --far-weight 3 \
  --output-prefix /tmp/joint-2000

simulate/run_polar_sweep.sh ESP32-S3-Si5351-Sample.wav /tmp/polar-sweep
```

`uv` supplies NumPy; `python3` works too if NumPy is installed. The full sweep
takes about four minutes. Each run writes `-target.wav`, `-receiver.wav` (ideal
transmitter), `-hw-receiver.wav` (hardware model), `-data.npz` (envelope and
phase at the update rate) and `-metrics.json`.

## Modes

| Mode | Drain envelope | Phase |
| --- | --- | --- |
| `phase-only` | constant | earlier alternating-projection optimizer, 4.5 kHz dump band |
| `classic` | low-passed analytic envelope | analytic phase |
| `joint` | band-limited by construction, optimized | free per sample, optimized |

`joint` minimizes a weighted spectral least-squares loss with Adam:

```text
sum_k w_k^2 |Z_k - D_k|^2,   Z = FFT(a * exp(j*phi)),   a = H * u
```

`D` is the wanted USB spectrum and zero everywhere else. There is no dump band.
`H` is the envelope low-pass, so the drain command never needs more bandwidth
than the modulator has, and `[floor, 1]` is a soft penalty. Weights are 1 in
the wanted band, `--stop-weight` on the opposite sideband, the carrier and the
adjacent channels, and `--far-weight` on everything further out. The weights
set the trade between in-band fidelity and emissions.

An earlier Gerchberg-Saxton-style projection version of `joint` stalled within
about 25 iterations and only gained 1--2 dB over `classic`. The gradient solver
replaced it.

## Hardware model

Every result is also played through a model of the real chain at 8x
oversampling:

- the Si5351 holds each frequency until the next update, so phase ramps
  linearly across an update interval instead of jumping;
- phase steps are int8 with error feedback, as in the firmware replay table;
- the drain command is 8-bit, zero-order held, and filtered by a
  second-order 10 kHz LC response (`--modulator-hz`);
- the PA leaks -50 dB of drive at zero drain (`--feedthrough`);
- the envelope is advanced against the phase over +/-60 us and the best
  opposite-sideband suppression is reported, along with the cost of being
  one update (31.2 us) off.

PA AM-AM/AM-PM distortion and modulator nonlinearity are not modelled yet.

## Results

10 s of `ESP32-S3-Si5351-Sample.wav`, 32 ksample/s, 300--3000 Hz USB, hardware
model, best alignment. OSB is opposite-sideband suppression; ACP is the worse
3 kHz adjacent channel; far-out is everything beyond that, all relative to the
wanted power. SI-SDR compares an ideal USB receiver's output with the
band-limited source.

| Mode | Envelope BW | SI-SDR | OSB | Carrier | ACP | Far-out | Wanted power |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| phase-only | (constant) | 17.3 dB | 20.9 dB | -38 dB | +0.1 dB | +2.6 dB | 21% |
| classic | 500 Hz | 17.4 dB | 19.9 dB | -37 dB | -22.5 dB | -30 dB | 98% |
| classic | 1 kHz | 20.2 dB | 21.1 dB | -41 dB | -23.6 dB | -31 dB | 98% |
| classic | 2 kHz | 29.4 dB | 29.1 dB | -50 dB | -32.0 dB | -36 dB | 100% |
| classic | 3 kHz | 44.2 dB | 41.6 dB | -64 dB | -40.8 dB | -41 dB | 100% |
| classic | 6 kHz | 46.0 dB | 50.6 dB | -67 dB | -50.6 dB | -45 dB | 100% |
| joint w3 | 500 Hz | 11.5 dB | 31.9 dB | -46 dB | -30.7 dB | -45 dB | 100% |
| joint w10 | 500 Hz | 9.3 dB | 42.1 dB | -57 dB | -40.5 dB | -47 dB | 100% |
| joint w3 | 1 kHz | 13.8 dB | 32.8 dB | -48 dB | -31.6 dB | -45 dB | 100% |
| joint w10 | 1 kHz | 11.2 dB | 43.5 dB | -59 dB | -42.1 dB | -47 dB | 100% |
| joint w3 | 2 kHz | 24.8 dB | 38.6 dB | -59 dB | -38.5 dB | -43 dB | 100% |
| joint w10 | 2 kHz | 18.9 dB | 44.8 dB | -65 dB | -44.4 dB | -46 dB | 100% |
| joint w3 | 3 kHz | 35.0 dB | 43.7 dB | -66 dB | -43.0 dB | -43 dB | 100% |
| joint w10 | 3 kHz | 24.2 dB | 47.2 dB | -67 dB | -47.2 dB | -46 dB | 100% |

Cost of misaligning the envelope by one update (31.2 us), hardware model:

| Mode | Envelope BW | OSB aligned | OSB one update off |
| --- | ---: | ---: | ---: |
| classic | 3 kHz | 41.6 dB | 31.6 dB |
| classic | 6 kHz | 50.6 dB | 32.4 dB |
| joint w10 | 1 kHz | 43.5 dB | 39.4 dB |
| joint w10 | 3 kHz | 47.2 dB | 32.8 dB |

## What the numbers say

1. **Phase-only does not survive the hardware.** Its ideal figures are
   excellent, but they rely on sample-exact phase jumps into the dump band.
   With the Si5351 ramping phase between updates, only 21% of the transmitted
   power lands in the wanted channel and the adjacent channels carry as much
   power as the wanted one. The modelled 20.9 dB OSB is in the same region as
   the 15.2 dB measured on air (that capture was the live DSP path, not the
   optimizer table).
2. **Hooking up the drain is the biggest single win.** Classic EER with a
   modulator that can follow 3 kHz goes to about 42 dB OSB and -41 dB ACP in
   the model.
3. **Alignment is worth about 10 dB.** The best envelope advance was 35 us in
   almost every run: half an update interval for the zero-order hold plus the
   LC filter's group delay. One update off costs 10--18 dB OSB at 3--6 kHz
   envelope bandwidth. A real modulator's delay has to be measured, which is
   what a closed loop through the Si4732 would do.
4. **Joint optimization buys emissions, paid for in fidelity.** At the same
   envelope bandwidth it gains 7--23 dB OSB and 6--20 dB ACP over classic, at
   lower SI-SDR. A 1 kHz modulator reaches classic-3 kHz emissions (43.5 dB
   OSB, -42 dB ACP), and its slow envelope is also less sensitive to
   misalignment. At 2--3 kHz, weight 3 is the balanced point: 38.6 dB OSB at
   24.8 dB SI-SDR, or 43.7 dB OSB at 35.0 dB SI-SDR.
5. **The hardware far-out floor is about -43 to -47 dB.** The ideal joint
   trajectories reach -50 to -63 dB, so the floor is set by the model's plain
   8-bit drain quantization and the int8 phase steps. The firmware's
   noise-shaped PWM should do better than this model.

## Next steps

- Listen to `-hw-receiver.wav` for the joint runs; SI-SDR below about 15 dB
  may still sound acceptable for SSB voice, or may not.
- Measure the real modulator's response and put it in `--modulator-hz`, or
  replace `butterworth2` with the measured response.
- Add PA AM-AM/AM-PM curves to the model, then run the optimizer through
  them, which is predistortion learned by the same solver.
- Export the joint trajectory (envelope bytes plus int8 phase steps) for a
  dummy-load replay, as `export_phase_firmware.py` does for phase-only.
