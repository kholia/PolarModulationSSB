# Polar SSB generator: ESP32-S3 + Si5351

This is the ESP-IDF port of the Pico 2 polar-modulation SSB proof of concept.
It keeps CAT/USB control on core 0 and pins the timing-critical modulation task
to core 1. ESP-IDF initializes and probes I2C0; the modulation path then uses a
small ESP32-S3 FIFO/command-register fast path so the wire transfer overlaps
the Hilbert transform and speech DSP.

## Why ESP-IDF

Use ESP-IDF, not the Arduino `Wire` API, for this firmware. The modulation loop
needs core affinity, maximum task priority, direct low-level control of I2C0,
and visibility into missed sample deadlines. Arduino-ESP32 is built on IDF but
its `Wire` compatibility layer cannot provide this phase-update path.

## Operating profiles

| CAT command | I2C SCL | Audio rate | Status |
| --- | ---: | ---: | --- |
| `*i1000*` | 1 MHz | 16 ksample/s | Reduced-speed fallback |
| `*i2000*` | 2 MHz | 16 ksample/s | Recommended fallback |
| `*i2032*` | 2 MHz | 32 ksample/s | Current default; verified low-level FIFO phase transport |
| `*i3032*` | 3 MHz | 32 ksample/s | Rejected on tested hardware; retained for experiments only |

Stop TX with `*t*` before changing profile, then use `*t*` again to restart.
The profile switch changes I2C timing, DSP coefficient banks, WAV decimation,
sample scheduling, and Si5351 phase scaling as one operation.

The 1--3 MHz profiles deliberately run beyond the published 400 kHz I2C limit. Use a
dedicated bus, very short traces, low capacitance, and external pull-ups. Start
around 1.5--2.2 kOhm at 3.3 V and verify SDA/SCL rise time, ACK, and actual SCL
frequency with a scope or logic analyzer. Do not rely on the ESP32-S3's weak
internal pull-ups. The 2 MHz profile must be validated on every board revision.

On the tested ESP32-S3/Si5351 board, the FIFO path sustained 32,000 phase
updates/s with no I2C errors. Average/max phase submission time was 2.5/26.2 us,
with two deadline misses in 480,007 samples. The generic ESP-IDF transaction
path reached only about 18,200 updates/s because it repeated substantial
per-transfer peripheral setup.

With drain modulation disabled, `*v*` toggles a phase-path self-test that
alternates between 700 Hz and 1900 Hz once per second. Embedded WAV speech is
the default source. Once every five seconds, the console reports the
phase-update count and measured writes/second, PLL `b_diff` range, deadline
jitter, and I2C submission or timeout errors. The lower report rate avoids
making USB console output a significant source of modulation-loop jitter.

## Default wiring

| Signal | GPIO |
| --- | ---: |
| Si5351 SDA | 1 |
| Si5351 SCL | 2 |
| PTT | 14 |
| Drain-modulation PWM | 4 |

The 25 MHz TCXO drives XA directly, so firmware configures the Si5351's
internal crystal load capacitance to 0 pF. Change pins, the TCXO value, or the boot profile with
`idf.py menuconfig` under **Polar SSB configuration**.

The amplitude output is a 320 kHz, 7-bit LEDC PWM that also clocks the
sample ticks (ten PWM periods per 32 ksample/s period). Each sample programs a
hardware linear fade, and `*dN*` aligns the envelope to the phase in 3.125 us
steps. Drain modulation is off at boot unless `POLAR_DRAIN_AM` is set; enable
it with `*a1*`. It requires the same external reconstruction/filter and drain
modulator expected by the Pico firmware. See
[QMX lessons and drain-AM timing](docs/qmx-lessons-and-drain-am.md).

The 32 ksample/s profile uses matched 31-tap halfband filters: input audio is
decimated to a 16 ksample/s speech/Hilbert core, and the resulting complex I/Q
vector is interpolated back to 32 ksample/s before amplitude and phase are
extracted. Runtime controls `*B2000*` through `*B3000*` select voice bandwidth,
`*L850*` sets the analytic-envelope threshold to 0.850, and `*R40*` selects a
4:1 envelope compression ratio. Stop TX before changing bandwidth.

## Build and flash

ESP-IDF 5.4 or newer is required.

```sh
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

The default Makefile build is containerized and copies the binaries to
`build-output/`:

```sh
make
```

It mounts the source tree at `/project`, so the host-side `build/` directory is
reused for fast incremental builds. It uses `espressif/idf:latest` by default.
Pin another image when needed with, for example,
`IDF_IMAGE=espressif/idf:v5.4.2 make`. Use `make local-build` when ESP-IDF is
installed locally. If `build/` was configured by a different IDF image or
Python environment, the build runs `idf.py fullclean` once and retries.

Flash the binaries in `build-output/` with the host `esptool`:

```sh
make flash                    # first /dev/cu.usbmodem* port, 460800 baud
make flash PORT=/dev/ttyACM0  # explicit port (BAUD=... also accepted)
make build-flash              # build, then flash
```

The USB Serial/JTAG console is enabled by default. When TX stops it prints the
number of full-period deadline misses and the worst observed lateness; use
those numbers when qualifying the 2 MHz profile.

## Measurement and tuning guides

- [Build, flash and monitor](docs/build-flash-monitor.md)
- [Natural-voice phase-only tuning](docs/phase-only-tuning.md)
- [Multirate and analytic-envelope DSP](docs/multirate-envelope-dsp.md)
- [Offline iterative phase-only optimizer](docs/offline-phase-optimizer.md)
- [Experimental optimized-phase firmware mode](docs/firmware-optimized-phase.md)
- [Joint envelope/phase optimizer](docs/joint-polar-optimizer.md)
- [QMX lessons and drain-AM timing](docs/qmx-lessons-and-drain-am.md)
- [Verification suite](simulate/verify/README.md)
- [Record macOS system audio](docs/macos-audio-capture.md)
- [Analyze an Airspy demodulated-audio capture](docs/airspy-audio-analysis.md)
- [Analyze raw Airspy IQ and the current baseline capture](docs/iq-capture-analysis.md)

The measured natural-voice values are the firmware defaults. Reapply them after
a runtime experiment with:

```sh
python3 scripts/polar-cat.py --port /dev/tty.usbmodem11301 natural
```
