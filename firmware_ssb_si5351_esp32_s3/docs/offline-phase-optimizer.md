# Offline iterative phase-only optimizer

This tool measures the quality ceiling of constant-envelope phase-only SSB. It
does not copy the instantaneous phase of the analytic audio. Instead, it
searches for a unit-amplitude complex sequence whose output through an ideal
USB receiver filter resembles the original band-limited voice.

## Listen to the included result

The repository contains a ten-second, 150-iteration result. On macOS, compare:

```sh
afplay simulate/phase-optimized-target.wav
afplay simulate/phase-optimized-baseline.wav
afplay simulate/phase-optimized-receiver.wav
```

- `target.wav` is the original voice band-limited to 300–3000 Hz.
- `baseline.wav` is direct analytic-phase normalization through the same ideal
  receiver.
- `receiver.wav` is the optimized constant-envelope sequence through that
  receiver.

All three listening files are independently normalized. The unnormalized
arrays and phase trajectory are stored in `phase-optimized-data.npz`; detailed
settings and measurements are in `phase-optimized-metrics.json`.

For the included ten-second run:

| Measurement | Direct phase baseline | Iterative result |
| --- | ---: | ---: |
| Correlation | 0.8785 | **0.9987** |
| SI-SDR | 5.29 dB | **25.82 dB** |
| Opposite-sideband suppression | 17.56 dB | **27.89 dB** |
| Carrier relative to wanted energy | -59.50 dB | **-76.99 dB** |

This confirms that the hollow/warbling sound is not an unavoidable consequence
of constant RF amplitude. It is largely a consequence of discarding the audio
envelope without encoding it elsewhere.

## Run it

The only Python dependency is NumPy:

```sh
python3 -m pip install numpy
```

Default ten-second run:

```sh
simulate/run_phase_optimizer.sh \
  ESP32-S3-Si5351-Sample.wav \
  simulate/phase-optimized \
  --iterations 150
```

Use a different source or longer duration:

```sh
simulate/run_phase_optimizer.sh source.wav /tmp/my-phase-result \
  --duration 20 \
  --iterations 250
```

Find an unconstrained ceiling at 32 ksample/s with a larger phase-noise dump
region:

```sh
simulate/run_phase_optimizer.sh source.wav /tmp/phase-32k \
  --sample-rate 32000 \
  --duration 10 \
  --iterations 200 \
  --dump-hz 4500
```

An experimental phase-slew constraint is available:

```sh
simulate/run_phase_optimizer.sh source.wav /tmp/phase-step-test \
  --max-phase-step 2.0
```

The strict 0.8-radian constraint used by the current real-time firmware did not
improve this optimization: it prevents the rapid phase alternation needed to
move envelope-coding noise above the receiver passband. A 2.0-radian constraint
gave a small improvement, while the unconstrained result uses phase steps near
pi.

## Algorithm

The optimizer applies alternating projections:

1. Resample and band-limit the desired audio.
2. Form its positive-frequency analytic spectrum in the wanted USB band.
3. Initialize a unit-amplitude sequence from the analytic phase.
4. Replace its wanted spectrum toward the desired spectrum.
5. Suppress DC, the opposite sideband and the receiver transition bands.
6. Leave a high-frequency dump band unconstrained.
7. Transform back to time and project every sample onto the unit circle.
8. Repeat and measure the ideal receiver output.

This effectively encodes envelope information as high-rate phase activity. An
SSB receiver removes that activity while retaining the desired in-band vector.

## RF safety and current limitations

Do **not** transmit the unconstrained trajectory over an antenna. In the
included result, about 95% of the constant-envelope energy is deliberately
moved above 4 kHz and phase steps approach pi radians/sample. It is an
algorithmic upper bound, not yet a normal-width amateur SSB waveform.

Before a firmware experiment, the next implementation must:

1. Replace the ideal brick-wall receiver with the measured Airspy passband.
2. Add occupied-bandwidth and adjacent-channel penalties.
3. Find the best compromise between phase slew, in-band reconstruction and
   opposite-sideband suppression, preferably at 32 ksample/s.
4. Quantize the phase increments using the exact Si5351 PLL mapping.
5. Test the precomputed sequence into a dummy load and inspect raw Airspy IQ.

Only after the raw-IQ spectrum is acceptable should a precomputed or real-time
version be enabled for radiated testing.

The precomputed dummy-load firmware mode is documented in
[`firmware-optimized-phase.md`](firmware-optimized-phase.md).
