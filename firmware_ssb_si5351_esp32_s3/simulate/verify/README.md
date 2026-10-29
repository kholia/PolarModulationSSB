# Firmware verification suite

Reproducible checks behind the October 2026 review of the ESP32-S3 polar SSB
firmware. Everything runs from this directory:

```sh
./run_verify.sh            # all steps
./run_verify.sh build      # IDF build into ../../build-verify (warning count)
./run_verify.sh dsp        # host tests: multirate OSB, loopback, drain_am unit test
./run_verify.sh timing     # Si5351 write-length / update-timing / alignment model
./run_verify.sh drain      # model of the LEDC-locked drain-AM firmware, delay sweep
./run_verify.sh twotone    # float32 two-tone generator time-base error
./run_verify.sh c5bench    # float DSP cost on a no-FPU RISC-V core (QEMU)
```

Requirements: Docker with `espressif/idf` (override with `IDF_IMAGE=...`),
a host C++ compiler, and `uv` (scripts run with `uv run --with numpy`).
Intermediate files go to `out/` (ignored by git).

## Files

| File | Purpose |
| --- | --- |
| `dump_dsp_stream.cpp` | Runs `tx_dsp_process()` on host; prints per-sample amplitude and phase step. `PM_LIMITER`, `PM_STEP`, `PM_KNEE`, `PM_RATIO` env vars override the polar tunables. |
| `firmware_am_model.py` | Mirrors the drain-AM TX loop: LEDC-locked ticks, variable-length Si5351 writes landing at STOP, one 10-step hardware fade per sample with carried remainder, and the `*dN*` delay split. Sweeps D and reports IMD and out-of-band power (`RATE`, `MOD_DEPTH`, `INT_DUTY=1`). |
| `../test_drain_am.cpp` | Unit test for `src/drain_am.h` (ramp carry bounds, delay split for every D at 16 k and 32 k). |
| `timing_model.py` | Reproduces `select_ssb_ms_div()`, the P1/P2 split, error-feedback `b_diff`, and `si5351_write_phase_fast()` suffix lengths per band. |
| `spec_model.py` | Continuous-time model of the Si5351 frequency steps (0.25 us grid) with an ideal drain envelope; reports two-tone IMD3/IMD5 and out-of-band power for several update-timing cases, and an amplitude-shift sweep (`FIXED_SHIFTS`). |
| `twotone_precision.py` | Phase error of the old unwrapped `t = index / rate` float32 two-tone generator. |
| `c5_softfloat_bench/` | IDF app (built for esp32c3, run in QEMU with `-icount 3`) that times the firmware DSP without an FPU. The QEMU counter reads 1.28x instructions (see the `CAL` line). |

## Results (IDF 6.1-dev, 14.2 MHz, 25 MHz TCXO, 2 MHz I2C, 32 ksample/s)

* Build: 0 warnings with `-Wall -Wextra`; image 985 KB.
* Host DSP: 53-104 dB opposite-sideband suppression from 300 to 2700 Hz.
* Write length: 1-4 data bytes (14-27 us at 2 MHz). In the model, this
  variable update instant and the 31/31/31/32 us tick pattern cost <= 0.1 dB.
  Fixed-length writes are not needed.
* Phase-step limiter (`pm_max_phase_step`, tanh at 0.4 rad/sample) with an
  ideal envelope: two-tone IMD3 -31.6 dBc. Disabled: -49.9 dBc. The
  firmware now clears `pm_phase_limiter` whenever drain AM is on.
* Amplitude/phase alignment, limiter off: aligned (+18 us) -47.9 dBc;
  current firmware pairing (amplitude ~49 us early) -24.3 dBc.
* 16 ksample/s, limiter off: -38.7 dBc (32 k is worth ~11 dB).
* New drain-AM loop (`firmware_am_model.py`): best at D = 6-8 steps
  (19-25 us), IMD3 -45 dBc and out-of-band -37 dB, against -24 dBc for the old
  pairing. Integer-only duty gives -39.6 dBc. 16 k peaks near -36 dBc.
  Details: [../../docs/qmx-lessons-and-drain-am.md](../../docs/qmx-lessons-and-drain-am.md).
* Two-tone generator (fixed in `main.cpp`): the old float time base was
  -42 dBc after 10 s and -27 dBc after 60 s.
* No-FPU RISC-V: ~22k instructions per sample. ESP32-C5 at 240 MHz would need
  ~100 us per sample against 31-62 us budgets; it would need a fixed-point DSP.

The drain modulator in `spec_model.py` is ideal (linear interpolation, no LC
group delay), so hardware alignment must be calibrated on air.
