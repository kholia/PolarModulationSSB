# QMX SSB lessons and drain-AM timing

This note covers an October 2026 review of this firmware against QRP Labs'
QMX SSB work, and the three changes that followed it:

- Hans Summers G0UPL, *Bringing SSB to QMX* (`../../datasheets/Bringing SSB to QMX.pdf`,
  also <https://qrp-labs.com/images/qmx/fdim/G0UPL.pdf>)
- <https://qrp-labs.com/qmxp/ssbbeta>

Every number below comes from host simulation or modelling, not RF
measurement. The scripts and exact commands are in
[`simulate/verify/`](../simulate/verify/README.md).

## What QMX does

| Topic | QMX | This firmware before the change |
| --- | --- | --- |
| DSP / phase-update rate | 12 ksample/s; MS5351M at 1 MHz I2C, about 75% bus occupancy | 32 ksample/s at 2 MHz I2C |
| Phase write | Fixed 4-register burst | 1-4 data bytes (shortest changed suffix) |
| Amplitude | 12-bit DAC, linear feedback modulator, 28x linear interpolation (2.98 us steps) | 8-bit PWM; software 12-step interpolation, disabled |
| Phase/amplitude sync | Sub-sample amplitude delay line, calibrated per band and per sideband by minimising IMD3+5+7 energy measured by its own receiver | Amplitude written about 1.5 samples early; no calibration |
| Phase-step limiting | None | tanh limiter at 0.4 rad/sample (32 k) |
| Low-level handling | Gate below about 1 Vpp, with hysteresis and fast attack (about 1 dB benefit) | Soft phase knee; RF gate compiled out |
| PA AM-PM | Per-band phase-vs-amplitude table, self-measured (about 2 dB IMD3 benefit) | None |
| Speech | Mic AGC, EQ, compression, CESSB (+4-5 dB talk power) | AGC, clip, filters, envelope compressor |
| Two-tone source | 120-point exact table, because generator spurs had limited measurements | Float time base that degraded over time |

QMX reports IMD3 of about -40.4 dB PEP (about -34 dBc) on 40 m with both
pre-distortions on. Badly set synchronisation gave about -15 dBc.

## Findings

All figures are from a 0.2 + 0.2 two-tone at 14.2 MHz, 32 ksample/s and
2 MHz I2C, with an ideal drain modulator unless stated otherwise.

1. **Variable-length writes and the 31/31/31/32 us tick pattern are harmless.**
   The Si5351 update lands 14-27 us after the tick, depending on length. That
   cost at most 0.1 dB, whether the chip latches per byte or at STOP. QMX's
   fixed-length burst is not needed.
2. **The phase limiter costs about 18 dB once amplitude is applied.** With
   ideal timing, IMD3 is -31.6 dBc with the limiter and -49.9 dBc without.
3. **Alignment matters most.** The old loop paired amplitude A[k-1] with the
   phase ramp that only reaches phase[k-1] one period later, plus the I2C
   transfer. That is about 49 us early, giving -24 dBc. Aligned, the model
   reaches -48 dBc. This agrees with QMX's sync findings and with
   [joint-polar-optimizer.md](joint-polar-optimizer.md).
4. **The old interpolation could not work at 32 k.** Its 12 x 3 us ramp is
   longer than the 31.25 us period, and its sub-steps were only sent while the
   loop was idle, so they came out in bursts after the DSP work.
5. **The two-tone generator drifted.** Its float32 time base left the source
   itself at -42 dBc after 10 s and -27 dBc after 60 s. Fixed: the index now
   wraps at the 100 Hz common period.
6. **32 k is worth keeping.** With the limiter off, 16 k reaches only
   -38.7 dBc against -49.9 dBc at 32 k.
7. **ESP32-C5 is not a drop-in replacement.** It has no FPU, and the float DSP
   costs about 22k instructions per sample (about 100 us at 240 MHz).

## The change: LEDC-locked drain AM

### Clocking

The drain PWM now runs at 320 kHz with 7-bit resolution from APB. 80 MHz /
(128 x 500/256) is exact with the LEDC fractional divider, so one 32 ksample/s
period is exactly ten 3.125 us PWM periods (twenty at 16 k). The PWM
channel's overflow counter raises a raw event every sample. The TX loop polls
it, so sample ticks sit exactly on PWM boundaries and the 31/32 us software
schedule is gone.

At TX start the loop checks that three consecutive events are one sample
apart. If they aren't, it logs an error and falls back to the old esp_timer
schedule; amplitude alignment is then only coarse.

### Envelope (fix 3)

Each sample programs one hardware linear fade: a start duty plus
`scale x steps`, in 1/16-LSB duty units. Because the integer scale can't
always land exactly on the target, the end point the hardware reaches is
carried into the next ramp (`drain_plan_ramp`). The result is a 10-step
continuous staircase, comparable to QMX's 28x interpolation at 12 k. The
one-pole envelope smoother (`*s*`) now defaults to 0.

### Alignment (fix 1)

`*dN*` sets the amplitude delay relative to the phase update in PWM periods
(3.125 us; range -60..80). Positive N holds the envelope back.
`drain_split_delay` turns N into:

- whole samples of history on whichever path must wait, and
- the PWM boundary after the tick on which the fade latches.

The loop writes the fade inside that boundary's window. The DSP runs in
whichever gap is larger, so the two never contend. Late writes are counted
as `amp_late`.

### Limiter (fix 2)

`pm_phase_limiter` is cleared whenever drain AM is on, and set again for
phase-only operation.

### Model result

Model: `simulate/verify/firmware_am_model.py`, 0.2 + 0.2 two-tone, mod depth 0.75.

| D (steps) | Delay | IMD3 | IMD5 | Out-of-band |
| ---: | ---: | ---: | ---: | ---: |
| -4 | -12.5 us | -36.4 dBc | -33.5 dBc | -26.3 dB |
| 0 | 0 us | -41.3 dBc | -37.7 dBc | -30.3 dB |
| **6 (default)** | 18.75 us | **-45.3 dBc** | -45.7 dBc | -37.2 dB |
| 8 | 25 us | -44.4 dBc | -46.9 dBc | -37.7 dB |
| 14 | 43.75 us | -38.6 dBc | -43.4 dBc | -31.3 dB |

The old loop's pairing gave -24.3 dBc. If the LEDC fractional duty bits turn
out not to dither, plain 7-bit duty gives -39.6 dBc at D=6. The 16 k profile
peaks near -36 dBc.

The model has no drain-filter group delay. A real LC filter delays the
envelope, which moves the optimum to a smaller D, possibly negative.

## Using it

```text
*t*        stop TX (delay and AM changes take effect on the next TX)
*i2032*    2 MHz / 32 ksample/s profile
*a1*       drain AM on (the phase limiter turns off automatically)
*d6*       amplitude delay = 6 x 3.125 us
*k* *w*    two-tone on and WAV off (WAV takes precedence when both are on)
*t*        start TX
```

The 5-second report and the end-of-TX line show `amp_late`, deadline misses
and the tick source (`LEDC` or `esp_timer`).

Boot defaults are set under **Polar SSB configuration** in menuconfig:
`POLAR_DRAIN_AM` (default off) and `POLAR_DRAIN_AM_DELAY_STEPS` (default 6).

## Verify on hardware before trusting the numbers

1. The boot log has no "drain PWM divider" error, and TX reports `LEDC` ticks.
2. On a scope, the PWM is 320.0 kHz. With a two-tone, the filtered drain
   envelope shows no step overshoot.
3. The LEDC fractional duty bits actually dither on the S3. Ramp the duty
   slowly and look for 1/16-LSB average steps. If they don't dither, expect
   about 6 dB less IMD3 margin.
4. `amp_late` stays at 0 at the D you use. The DSP must fit in the larger gap
   around the fade write.
5. Sweep `*dN*` on a dummy load with the two-tone and keep the D with the
   lowest IMD3+5+7 energy. Repeat for each band and for LSB, as QMX does.

## Calibrating with the on-board Si4732 (proposed, not implemented)

QMX listens to its own transmitter through T/R-switch leakage. It shifts its
receive LO by +2700 Hz, so IMD3/5/7 of a 700 + 1900 Hz USB two-tone fall at
400/1600/2800 Hz audio, then sums the audio power for 1 s at each delay step.

Two ways to do the same with the Si4732:

- **SSB mode (needs the SSB patch).** Same method as QMX. This needs the
  Si4732 audio output on an ESP32 ADC pin.
- **RSSI only, no audio path.** Use AM mode at the narrowest bandwidth (1 kHz),
  tuned to IMD5/IMD7 (for example the carrier +5 kHz). Average many
  `GET_RSQ_STATUS` RSSI reads (1 dB steps) per delay and minimise. This is
  coarser but needs no extra wiring.

Either way the Si4732 must be on its own I2C bus or controller. It is
specified to 400 kHz, and the Si5351 bus carries 2-3 MHz traffic during TX.
Its RF input must also be attenuated enough not to overload.
