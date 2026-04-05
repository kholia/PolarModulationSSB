# Airspy demodulated-audio analysis

## Reproduce the report

Install the optional analysis tools on macOS:

```sh
brew install ffmpeg sox
```

Analyze an Airspy WAV capture:

```sh
scripts/analyze-airspy-capture.sh output.wav analysis-output
```

The report directory contains:

- `metadata.txt` — codec, sample rate, channel count, duration and size
- `levels.txt` — FFmpeg peak/RMS and clipping statistics
- `spectrogram.png` — time/frequency view of the received audio
- `timing-and-levels.txt` — dependency-free timing, envelope and transient data
- `band-levels.tsv` — repeatable RMS measurements by speech band

For only the dependency-free checks:

```sh
python3 scripts/analyze-airspy-audio.py output.wav \
  --original path/to/original.wav \
  --speech-dsp path/to/speech_dsp_out.wav \
  --phase-sim path/to/loopback_out.wav
```

## Findings for `output.wav`

The checked-in capture is 48 kHz, 16-bit stereo and 19.066 seconds long. Its
left and right channels are bit-identical. Peak level is -9.11 dBFS and RMS is
-15.15 dBFS, so the file itself is not clipping.

The embedded phrase repeats after 9.171 seconds with envelope correlation
0.953. Its playback-rate estimate against the source is 0.999, confirming that
the firmware's 16 ksample/s scheduling and perceived pitch are now correct.

The capture's median-to-95th-percentile frame RMS span is only about 1.25 dB.
That very flat received envelope is expected to some degree from phase-only RF,
but it also explains the dense, processed quality. Relative to the 1–2 kHz
band, the capture and phase-only simulator both place 3–4.2 kHz at -9.30 dB.
That close match validates the simulator and implicates the transmitter DSP
rather than the SDR recording path as the dominant source of coloration.

The recommended follow-up is to apply the natural-voice preset in
[`phase-only-tuning.md`](phase-only-tuning.md), make a new capture under the
same receiver bandwidth and gain, and compare both reports.
