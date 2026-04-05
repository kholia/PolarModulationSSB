# Multirate and analytic-envelope DSP

The 32 ksample/s transport profile now keeps the expensive speech and Hilbert
analysis at 16 ksample/s. This preserves a fast Si5351 phase-update stream
without executing the complete analytic chain twice as often.

## Signal path

At 16 ksample/s the path remains:

```text
audio -> speech DSP -> Hilbert I/Q -> envelope compressor -> polar encoder
```

At 32 ksample/s it becomes:

```text
32k audio -> halfband decimator -> 16k speech DSP -> 16k Hilbert I/Q
          -> envelope compressor -> complex halfband interpolator
          -> 32k polar encoder -> Si5351 phase updates
```

The 31-tap halfband filters require only eight symmetric tap-pair multiplies
plus one centre multiply. I and Q use identical interpolation filters. Phase is
calculated only after interpolation; wrapped `atan2` angles are never
interpolated.

The low-rate analysis is also pipelined across two 32 ksample/s ticks. The
decimator and speech processor run on the first tick; Hilbert/envelope analysis
and complex interpolation run on the second while an older prepared I/Q pair is
transmitted. This balances the worst-case work inside the 31 microsecond sample
deadline at the cost of one additional 62.5 microsecond I/Q-pair delay.

The 127-tap Type-III Hilbert transformer similarly exploits its zero taps and
antisymmetry. Each pair is evaluated as `h[k] * (x[n-k] - x[n-126+k])`, reducing
the kernel from 62 to 31 multiplies per analytic sample.

## Envelope compressor

Compression operates on `sqrt(I*I + Q*Q)`, then applies the same gain to both
components. It therefore changes envelope without rotating the analytic vector.
A 10% quadratic knee joins unity gain to the selected ratio, and an absolute
0.98 ceiling is applied again after interpolation to catch FIR overshoot.

Defaults and CAT controls:

| Control | Default | CAT command | Range |
| --- | ---: | --- | --- |
| Envelope threshold | 0.850 | `*L850*` | 0.050–1.500 |
| Compression ratio | 4:1 | `*R40*` | 1:1–20:1 |
| Envelope ceiling | 0.980 | compile-time | fixed |

A 1:1 ratio disables compression. With drain AM disabled, equal I/Q scaling
does not alter phase-only RF except where the final low-level phase knee is
involved.

## Voice bandwidth banks

`*B2000*`, `*B2200*`, `*B2400*`, `*B2700*`, and `*B3000*` select matched
pre/post speech low-pass coefficients. The default remains 2700 Hz. Stop TX
before changing bandwidth so all filter and interpolation state can restart
cleanly. The 3000 Hz bank is experimental because it approaches the upper edge
of the 16 ksample/s Hilbert design.

## Regression checks

Run the focused tone/image and envelope-ceiling test:

```sh
simulate/run_multirate_test.sh
```

Run comparable full loopbacks:

```sh
simulate/run_loopback.sh ESP32-S3-Si5351-Sample.wav 16000
simulate/run_loopback.sh ESP32-S3-Si5351-Sample.wav 32000
SIMULATE_DRAIN_AM=1 simulate/run_loopback.sh ESP32-S3-Si5351-Sample.wav 32000
```
