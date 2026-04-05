# Phase-only voice tuning

The recommended first A/B preset is:

| Control | Previous profile | New firmware default |
| --- | ---: | ---: |
| Pre-emphasis | 0.70 | **0.00** |
| Clip level | 0.40 | **0.40** |
| AGC target | 0.35 | **0.15** |
| AGC maximum gain | 3.0 | **1.5** |
| Phase limit | 2.0 rad/sample | **0.8 rad/sample** |
| Low-level phase knee | 0.0020 | **0.0002** |

These measured values are now the compile-time defaults. Receiver bandwidth,
demodulator tuning, and the particular voice still matter, so the previous
profile remains available for an immediate on-air A/B comparison.

## Apply and undo the preset

Install PySerial once:

```sh
python3 -m pip install pyserial
```

Select the 16 ksample/s profile first because a profile change resets the phase
limit. Stop TX, apply the profile and preset, then restart TX:

```sh
python3 scripts/polar-cat.py --port /dev/tty.usbmodem11301 '*t*' '*i2000*' natural '*t*'
```

Restore the previous firmware values with:

```sh
python3 scripts/polar-cat.py --port /dev/tty.usbmodem11301 '*t*' legacy '*t*'
```

The equivalent natural-preset CAT stream is:

```text
*A150**G15**C400**E0**B2700**L850**R40**e8**j2*
```

CAT overrides are volatile. A reboot returns to the natural-voice defaults.

## Why this preset

The Airspy capture showed no ADC/file clipping, but the received waveform had
a nearly constant frame envelope and more upper-frequency energy than the
source. The offline phase-only model closely reproduced the capture's relative
2–4.2 kHz spectrum. This points to the constant-envelope phase conversion and
its conditioning—not the Airspy audio path—as the main source of coloration.

The joint sweep found that:

- Removing pre-emphasis preserves the source timbre and reduces the metallic
  consonant edge.
- A lower AGC target and gain ceiling keep most speech below the unchanged 0.4
  clip threshold. The clipper remains as protection, but stops dominating the
  sound.
- A 0.8 rad/sample phase limit suppresses large instantaneous phase jumps.
- A small 0.0002 knee holds phase only extremely close to analytic-envelope
  zero, instead of flattening legitimate low-level speech detail.

On the embedded reference phrase, the joint correlation score increased from
0.4866 to 0.5161. On the longer loopback input it increased from 0.4025 to
0.4158, while excess 3–4 kHz energy fell by about 3.6 dB. The gentler linear
filter-only path scored 0.3739, so bypassing the speech topology altogether was
not selected.

Phase-only SSB is intrinsically constant-envelope at the Si5351 output. No
parameter choice can reproduce the source amplitude envelope exactly; these
values aim to preserve pitch, formants, cadence, and spectral balance while
avoiding unnecessary clipping and phase bursts.

## Reproduce the sweeps

Run the one-factor sweep:

```sh
simulate/run_phase_tuner.sh --one-factor simulate/phase_tuning_one_factor.csv
```

Run the staged joint sweep:

```sh
simulate/run_phase_tuner.sh --joint simulate/phase_tuning_joint.csv
```

The joint run searches 300 speech-stage combinations, keeps the best 12, then
sweeps phase limiting and the low-level knee for those finalists. The CSV
contains every reported value and its lag-compensated Pearson correlation.
Correlation is a regression proxy; the final criterion remains an RF A/B
capture and listening test.

## Generate comparable loopbacks

Baseline:

```sh
POLAR_AGC_TARGET=0.35 \
POLAR_AGC_MAX_GAIN=3.0 \
POLAR_CLIP_LEVEL=0.4 \
POLAR_PREEMPH=0.7 \
POLAR_BANDWIDTH=2700 \
POLAR_ENV_RATIO=1 \
POLAR_PHASE_LIMIT=2.0 \
POLAR_SOFT_KNEE=0.002 \
POLAR_OUTPUT_WAV=/tmp/polar-baseline.wav \
POLAR_SPEECH_OUTPUT_WAV=/tmp/polar-baseline-speech.wav \
simulate/run_loopback.sh input.wav 16000
```

Natural preset:

```sh
POLAR_OUTPUT_WAV=/tmp/polar-natural.wav \
POLAR_SPEECH_OUTPUT_WAV=/tmp/polar-natural-speech.wav \
simulate/run_loopback.sh input.wav 16000
```

Set `SIMULATE_DRAIN_AM=1` only when intentionally evaluating a future drain-AM
path. The current hardware/firmware configuration is phase-only, so the
simulator defaults to fixed RF amplitude.
