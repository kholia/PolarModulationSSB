# Airspy IQ capture analysis

## Analyze a recording

`scripts/analyze-complex-iq.py` accepts interleaved little-endian complex
float32 IQ. The Airspy HF+ recording in `0.raw` is consistent with its native
768 ksample/s rate:

```sh
python3 scripts/analyze-complex-iq.py 0.raw \
  --sample-rate 768000 \
  --output-dir analysis-live-baseline
```

This writes `iq-metrics.json` and a ±40 kHz `iq-spectrum.tsv`. Add `--plot` if
Matplotlib is installed and a PNG is wanted.

## Results for `0.raw` and `o.wav`

The IQ file contains 13,990,702 complex float32 samples, or 18.2171 seconds at
768 ksample/s. The corresponding audio file is 48 kHz, stereo PCM and 18.7686
seconds long; its channels are bit-identical.

RF measurements relative to the detected carrier:

| Measurement | Result |
| --- | ---: |
| Carrier frequency offset | -41.02 Hz |
| Wanted USB band | +300 to +3000 Hz |
| Opposite-sideband suppression | 15.22 dB |
| Carrier relative to integrated wanted power | -8.28 dB |
| 3.0–4.5 kHz transition energy | -22.21 dB |
| 4.5–8.0 kHz combined energy | -29.13 dB |
| First ±16 kHz update images | about -34.2 dB |
| Noise-subtracted 90% occupied bandwidth | 1.816 kHz |
| Noise-subtracted 95% occupied bandwidth | 2.235 kHz |
| Noise-subtracted 99% occupied bandwidth | 4.603 kHz |

The -41 Hz offset is the combined transmitter/receiver frequency error; it
cannot be attributed to the 25 MHz TCXO alone without an independently locked
Airspy reference.

The audio is not clipping: peak is -9.10 dBFS and RMS is -14.94 dBFS. Its
3–4.2 kHz energy is about 41.5 dB below the 1–2 kHz band, consistent with the
narrow receiver filter and the absence of the previous upper-band hash.

## Important mode identification

This capture is the **live DSP path**, not the precomputed optimizer path:

- `o.wav` repeats after 9.171 seconds with 0.984 envelope correlation.
- Correlation at the optimized trajectory's exact 10.000-second loop is only
  0.103.
- IQ energy in the optimizer's 4.5–8 kHz dump band is 29.1 dB below the wanted
  band rather than dominating it.

This makes the recording a valuable live-path baseline, and also shows that
the natural DSP changes plus the narrower receiver passband eliminated the
audible `whun-whun` artifact.

## Capture the optimized mode

After flashing the latest image, explicitly enable the mode:

```sh
python3 scripts/polar-cat.py \
  --port /dev/tty.usbmodem11301 \
  optimized
```

Reconnect the terminal and confirm this exact line before recording:

```text
[PHASE-OPT] Experimental 10 s trajectory; wideband dump energy; dummy-load testing only
```

The optimized capture must repeat at exactly 10.000 seconds. Save it under new
names, for example `optimized.raw` and `optimized.wav`, so it can be compared
directly with this baseline.
