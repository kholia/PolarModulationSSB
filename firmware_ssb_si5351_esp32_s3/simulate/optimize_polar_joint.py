#!/usr/bin/env python3
# /// script
# requires-python = ">=3.9"
# dependencies = ["numpy>=1.24"]
# ///
"""Jointly optimize drain envelope and Si5351 phase for polar SSB.

optimize_phase_only.py showed that constant-envelope SSB can sound right, but
only by pushing ~95% of the RF energy into a wideband "dump" band above the
receiver passband. That is not a transmittable waveform.

Here the drain modulator carries a slow envelope and the phase path carries
the rest. Alternating projections enforce, every iteration:

  frequency domain: wanted USB bins move toward the target; every other bin
                    (opposite sideband, carrier, adjacent channels, far out)
                    is suppressed. There is no dump band.
  time domain:      z[n] = a[n] * exp(j*phi[n]) where a is band-limited to
                    what the envelope modulator can follow, stays inside
                    [floor, 1], and phi optionally obeys a slew limit.

Three modes share the same metrics so they can be compared directly:

  classic     textbook EER: a = low-passed |analytic|, phi = angle(analytic)
  phase-only  the earlier constant-envelope optimizer (with its dump band)
  joint       this optimizer

Every result is also evaluated through a hardware model: Si5351 frequency
held between updates (linear phase), int8 phase steps with error feedback,
8-bit drain PWM through a second-order LC filter, PA feedthrough at zero
drain, and a search over envelope/phase timing alignment.
"""

import argparse
import json
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from optimize_phase_only import (  # noqa: E402
    correlation, project_phase_slew, read_pcm_wav, resample, si_sdr,
    wrap_phase, write_pcm16)


def raised_cosine_lowpass(frequencies, cutoff_hz, transition_hz):
    """Zero-phase low-pass mask; a soft edge limits time-domain ringing."""
    f = np.abs(frequencies)
    low = cutoff_hz - transition_hz / 2
    mask = np.clip((cutoff_hz + transition_hz / 2 - f) / transition_hz, 0, 1)
    mask = 0.5 - 0.5 * np.cos(np.pi * mask)
    mask[f <= low] = 1.0
    return mask


def butterworth2(frequencies, cutoff_hz):
    """Second-order Butterworth response, e.g. a drain modulator's LC filter.

    Unlike the optimizer's zero-phase mask this is causal: its group delay is
    what the timing-alignment search has to find.
    """
    s = 1j * frequencies / cutoff_hz
    return 1.0 / (s * s + np.sqrt(2) * s + 1)


def band_energy(power, frequencies, low_hz, high_hz):
    return float(np.sum(power[(frequencies >= low_hz) & (frequencies <= high_hz)]))


def db(ratio):
    return float(10 * np.log10(ratio + 1e-30))


def spectral_metrics(z, rate, low_hz, high_hz):
    """RF-side figures: where the transmitted energy actually goes."""
    spectrum = np.fft.fft(z)
    power = np.abs(spectrum) ** 2
    f = np.fft.fftfreq(len(z), 1 / rate)
    total = float(np.sum(power))
    wanted = band_energy(power, f, low_hz, high_hz)
    width = high_hz  # one SSB channel's width, for the adjacent channels
    opposite = band_energy(power, f, -high_hz, -low_hz)
    carrier = band_energy(power, f, -20.0, 20.0)
    upper_adjacent = band_energy(power, f, high_hz, high_hz + width)
    lower_adjacent = band_energy(power, f, -high_hz - width, -high_hz)
    far = float(np.sum(power[(f > high_hz + width) | (f < -high_hz - width)]))
    order = np.argsort(f)
    cumulative = np.cumsum(power[order]) / total
    occupied = (f[order][np.searchsorted(cumulative, 0.995)] -
                f[order][np.searchsorted(cumulative, 0.005)])
    return {
        "wanted_power_percent": 100 * wanted / total,
        "opposite_sideband_suppression_db": db(wanted / (opposite + 1e-30)),
        "carrier_relative_db": db(carrier / wanted),
        "upper_adjacent_db": db(upper_adjacent / wanted),
        "lower_adjacent_db": db(lower_adjacent / wanted),
        "far_out_db": db(far / wanted),
        "occupied_bandwidth_99_hz": float(occupied),
    }


def audio_metrics(z, rate, wanted_mask, target, decimation=1):
    """What an ideal USB receiver hears, against the band-limited source."""
    received = np.real(np.fft.ifft(np.fft.fft(z) * wanted_mask))[::decimation]
    received = received[:len(target)]
    return {
        "correlation": correlation(target, received),
        "si_sdr_db": si_sdr(target, received),
    }, received


def quantize_steps(steps):
    """int8 phase steps with error feedback, as the firmware trajectory uses."""
    scale = np.pi / 127.0
    codes = np.empty(len(steps))
    residual = 0.0
    for index, step in enumerate(steps):
        shaped = step + residual
        code = min(max(round(shaped / scale), -127), 127)
        codes[index] = code
        residual = shaped - code * scale
    return codes * scale


def hardware_waveform(amplitude, phase, oversample, rate, modulator_hz,
                      amplitude_bits, feedthrough, advance):
    """Continuous-time-ish RF envelope as the real chain would produce it.

    The Si5351 holds each frequency until the next update, so phase moves
    linearly across the sample interval rather than jumping. The drain PWM
    holds each duty value and its LC filter smooths and delays it. `advance`
    (in oversampled ticks) shifts the envelope command earlier to line it up.
    """
    steps = quantize_steps(wrap_phase(np.diff(phase, append=phase[0])))
    ramp = np.arange(oversample) / oversample
    phase_hw = (phase[0] + np.concatenate(([0.0], np.cumsum(steps[:-1]))))
    phase_hw = (phase_hw[:, None] + steps[:, None] * ramp[None, :]).ravel()

    levels = (1 << amplitude_bits) - 1
    duty = np.round(np.clip(amplitude, 0, 1) * levels) / levels
    held = np.repeat(duty, oversample)
    f = np.fft.fftfreq(len(held), 1 / (rate * oversample))
    drain = np.real(np.fft.ifft(np.fft.fft(held) * butterworth2(f, modulator_hz)))
    drain = np.roll(drain, -advance)
    # A switching PA still passes some drive through Crss at zero drain.
    return (np.maximum(drain, 0) + feedthrough) * np.exp(1j * phase_hw)


def optimize_gradient(amplitude, phase, desired_spectrum, weights,
                      envelope_mask, args):
    """Adam on a weighted spectral least-squares loss.

    Minimizes sum_k w_k^2 |Z_k - D_k|^2 with Z = FFT(a * exp(j*phi)) and D
    zero outside the wanted band, so the weights trade in-band fidelity
    against opposite sideband, adjacent-channel and far-out energy directly.
    phi is free per sample. a = H*u with H the envelope low-pass, so the
    drain command is band-limited by construction; [floor, 1] is a soft
    penalty, clipped hard at the end.
    """
    count = len(phase)
    floor = args.envelope_floor
    weights2 = weights ** 2
    norm = float(np.sum(weights2 * np.abs(desired_spectrum) ** 2))

    def lowpass(x):
        return np.real(np.fft.ifft(np.fft.fft(x) * envelope_mask))

    u = amplitude.copy()
    params = [phase.copy(), u]
    moments = [[np.zeros(count), np.zeros(count)] for _ in params]
    rates = [args.phase_rate, args.amplitude_rate]
    beta1, beta2 = 0.9, 0.999

    for iteration in range(1, args.iterations + 1):
        phase, u = params
        amplitude = lowpass(u)
        carrier = np.exp(1j * phase)
        z = amplitude * carrier
        error = np.fft.fft(z) - desired_spectrum
        loss = float(np.sum(weights2 * np.abs(error) ** 2)) / norm
        # dL/dz* = F^H W^2 (Fz - D); numpy's ifft carries a 1/N.
        g = count * np.fft.ifft(weights2 * error) / norm
        grad_phase = 2 * np.real(np.conj(g) * 1j * z)
        grad_amplitude = 2 * np.real(np.conj(g) * carrier)
        below = np.minimum(amplitude - floor, 0)
        above = np.maximum(amplitude - 1.0, 0)
        loss += args.bound_penalty * float(np.sum(below ** 2 + above ** 2)) / count
        grad_amplitude += 2 * args.bound_penalty * (below + above) / count
        grad_u = lowpass(grad_amplitude)  # H is symmetric (zero phase)

        for index, gradient in enumerate((grad_phase, grad_u)):
            m, v = moments[index]
            m *= beta1
            m += (1 - beta1) * gradient
            v *= beta2
            v += (1 - beta2) * gradient * gradient
            m_hat = m / (1 - beta1 ** iteration)
            v_hat = v / (1 - beta2 ** iteration)
            params[index] = params[index] - rates[index] * m_hat / (np.sqrt(v_hat) + 1e-12)
        if args.max_phase_step > 0:
            params[0] = project_phase_slew(params[0], args.max_phase_step)

        if iteration % args.report_every == 0 or iteration == args.iterations:
            z = np.clip(lowpass(params[1]), floor, 1.0) * np.exp(1j * params[0])
            spectral = spectral_metrics(z, args.sample_rate, args.low_hz, args.high_hz)
            print(f"iteration={iteration:4d} loss={loss:.3e} "
                  f"OSB={spectral['opposite_sideband_suppression_db']:5.1f} dB "
                  f"ACP={max(spectral['upper_adjacent_db'], spectral['lower_adjacent_db']):6.1f} dB "
                  f"far={spectral['far_out_db']:6.1f} dB", flush=True)

    phase, u = params
    return np.clip(lowpass(u), floor, 1.0), phase


def optimize(mode, desired, desired_spectrum, wanted, keep, envelope_mask,
             args):
    floor = args.envelope_floor
    target_envelope = np.abs(desired)
    phase = np.angle(desired)
    # Absolute phase during true silence is arbitrary; hold it rather than
    # seeding atan2 noise (same choice as optimize_phase_only.py).
    quiet = np.flatnonzero(target_envelope < args.target_peak * 1e-4)
    for index in quiet:
        if index:
            phase[index] = phase[index - 1]

    def band_limit(envelope):
        limited = np.real(np.fft.ifft(np.fft.fft(envelope) * envelope_mask))
        return np.clip(limited, floor, 1.0)

    if mode == "phase-only":
        amplitude = np.ones_like(target_envelope)
    else:
        amplitude = band_limit(target_envelope)
    z = amplitude * np.exp(1j * phase)
    if mode == "classic":
        return amplitude, phase
    if mode == "joint":
        f = np.fft.fftfreq(len(z), 1 / args.sample_rate)
        weights = np.full(len(z), args.far_weight)
        stop = ((f >= -args.high_hz) & (f < args.low_hz)) | (
            (f > args.high_hz) & (f <= 2 * args.high_hz)) | (
            (f < -args.high_hz) & (f >= -2 * args.high_hz))
        weights[stop] = args.stop_weight
        weights[wanted] = 1.0
        return optimize_gradient(amplitude, phase, desired_spectrum, weights,
                                 envelope_mask, args)

    for iteration in range(1, args.iterations + 1):
        spectrum = np.fft.fft(z)
        spectrum[wanted] = ((1 - args.spectral_relaxation) * spectrum[wanted] +
                            args.spectral_relaxation * desired_spectrum[wanted])
        spectrum[~keep] *= args.stopband_retention
        candidate = np.fft.ifft(spectrum)
        phase = project_phase_slew(np.angle(candidate), args.max_phase_step)
        if mode == "joint":
            amplitude = band_limit(np.abs(candidate))
        z = amplitude * np.exp(1j * phase)
        if iteration % args.report_every == 0 or iteration == args.iterations:
            spectral = spectral_metrics(z, args.sample_rate, args.low_hz, args.high_hz)
            print(f"iteration={iteration:4d} "
                  f"wanted={spectral['wanted_power_percent']:5.1f}% "
                  f"OSB={spectral['opposite_sideband_suppression_db']:5.1f} dB "
                  f"ACP={max(spectral['upper_adjacent_db'], spectral['lower_adjacent_db']):6.1f} dB "
                  f"far={spectral['far_out_db']:6.1f} dB", flush=True)
    return amplitude, phase


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", help="PCM WAV containing the source voice")
    parser.add_argument("--mode", choices=("joint", "classic", "phase-only"),
                        default="joint")
    parser.add_argument("--output-prefix", default="simulate/polar-joint")
    parser.add_argument("--sample-rate", type=int, default=32000,
                        help="Si5351/drain update rate")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="seconds to optimize; 0 uses the entire file")
    parser.add_argument("--iterations", type=int, default=150)
    parser.add_argument("--low-hz", type=float, default=300.0)
    parser.add_argument("--high-hz", type=float, default=3000.0)
    parser.add_argument("--dump-hz", type=float, default=4500.0,
                        help="phase-only mode: dump band starts here")
    parser.add_argument("--envelope-bandwidth", type=float, default=3000.0,
                        help="Hz the drain envelope may occupy (joint/classic)")
    parser.add_argument("--envelope-floor", type=float, default=0.01,
                        help="lowest drain amplitude, relative to full scale")
    parser.add_argument("--target-peak", type=float, default=0.90,
                        help="peak desired analytic envelope, below unity")
    parser.add_argument("--spectral-relaxation", type=float, default=0.65)
    parser.add_argument("--stopband-retention", type=float, default=0.02)
    parser.add_argument("--max-phase-step", type=float, default=0.0,
                        help="radians/sample; 0 means unconstrained")
    parser.add_argument("--report-every", type=int, default=25)
    joint = parser.add_argument_group("joint mode (gradient solver)")
    joint.add_argument("--stop-weight", type=float, default=3.0,
                       help="weight on opposite sideband, carrier and adjacent "
                            "channels relative to the wanted band")
    joint.add_argument("--far-weight", type=float, default=1.0,
                       help="weight on everything beyond the adjacent channels")
    joint.add_argument("--phase-rate", type=float, default=0.02,
                       help="Adam step size for phase (radians)")
    joint.add_argument("--amplitude-rate", type=float, default=0.002,
                       help="Adam step size for the envelope command")
    joint.add_argument("--bound-penalty", type=float, default=100.0,
                       help="soft penalty keeping the envelope in [floor, 1]")
    hardware = parser.add_argument_group("hardware model")
    hardware.add_argument("--oversample", type=int, default=8)
    hardware.add_argument("--modulator-hz", type=float, default=10000.0,
                          help="drain modulator LC cutoff (2nd-order)")
    hardware.add_argument("--amplitude-bits", type=int, default=8)
    hardware.add_argument("--feedthrough", type=float, default=0.003,
                          help="RF leakage at zero drain (0.003 = -50 dB)")
    hardware.add_argument("--max-advance-us", type=float, default=60.0,
                          help="envelope/phase alignment search range")
    args = parser.parse_args()

    nyquist = args.sample_rate / 2
    if not 0 < args.low_hz < args.high_hz < nyquist:
        parser.error("require 0 < low-hz < high-hz < Nyquist")
    if not 0 < args.target_peak <= 1:
        parser.error("--target-peak must be in (0, 1]")

    source_rate, source = read_pcm_wav(args.input)
    if args.duration > 0:
        source = source[:round(args.duration * source_rate)]
    source = resample(source, source_rate, args.sample_rate)
    source -= np.mean(source)
    count = len(source)
    # Circular processing without zero padding: fade the ends so the seam is
    # silent. The trajectory can then loop like the firmware's replay table.
    edge = min(round(0.01 * args.sample_rate), count // 2)
    fade = np.sin(np.linspace(0, np.pi / 2, edge)) ** 2
    source[:edge] *= fade
    source[count - edge:] *= fade[::-1]

    f = np.fft.fftfreq(count, 1 / args.sample_rate)
    wanted = (f >= args.low_hz) & (f <= args.high_hz)
    keep = wanted | (np.abs(f) >= args.dump_hz) if args.mode == "phase-only" else wanted
    desired_spectrum = np.zeros(count, dtype=np.complex128)
    desired_spectrum[wanted] = 2.0 * np.fft.fft(source)[wanted]
    desired = np.fft.ifft(desired_spectrum)
    peak = np.max(np.abs(desired))
    if peak == 0:
        raise SystemExit("source has no energy in the requested speech band")
    desired_spectrum *= args.target_peak / peak
    desired = np.fft.ifft(desired_spectrum)
    target = np.real(desired)
    envelope_mask = raised_cosine_lowpass(
        f, args.envelope_bandwidth, 0.25 * args.envelope_bandwidth)

    print(f"mode={args.mode} rate={args.sample_rate} samples={count} "
          f"envelope_bw={args.envelope_bandwidth:.0f} Hz "
          f"floor={args.envelope_floor} iterations={args.iterations}", flush=True)
    amplitude, phase = optimize(args.mode, desired, desired_spectrum, wanted,
                                keep, envelope_mask, args)
    z = amplitude * np.exp(1j * phase)

    ideal_audio, ideal_received = audio_metrics(z, args.sample_rate, wanted, target)
    steps = wrap_phase(np.diff(phase))
    ideal = {
        **ideal_audio,
        **spectral_metrics(z, args.sample_rate, args.low_hz, args.high_hz),
        "mean_drain_power": float(np.mean(amplitude ** 2)),
        "phase_step_rms": float(np.sqrt(np.mean(steps ** 2))),
        "phase_step_p99": float(np.percentile(np.abs(steps), 99)),
    }

    # Hardware model, scanning the envelope advance for the best alignment.
    hw_rate = args.sample_rate * args.oversample
    hw_f = np.fft.fftfreq(count * args.oversample, 1 / hw_rate)
    hw_wanted = (hw_f >= args.low_hz) & (hw_f <= args.high_hz)
    max_ticks = round(args.max_advance_us * 1e-6 * hw_rate)
    tick_us = 1e6 / hw_rate

    def evaluate(advance):
        z_hw = hardware_waveform(amplitude, phase, args.oversample,
                                 args.sample_rate, args.modulator_hz,
                                 args.amplitude_bits, args.feedthrough, advance)
        audio, received = audio_metrics(z_hw, hw_rate, hw_wanted, target,
                                        args.oversample)
        return {**audio, **spectral_metrics(z_hw, hw_rate, args.low_hz,
                                            args.high_hz)}, received

    if args.mode == "phase-only":
        sweep = {0: evaluate(0)}  # constant drain: alignment is irrelevant
    else:
        sweep = {}
        for advance in range(-max_ticks, max_ticks + 1, max(1, args.oversample // 4)):
            sweep[advance] = evaluate(advance)
    best = max(sweep, key=lambda k: sweep[k][0]["opposite_sideband_suppression_db"])
    hw_best, hw_received = sweep[best]
    one_sample = args.oversample
    misaligned = {}
    for offset in (-one_sample, one_sample):
        if args.mode != "phase-only":
            misaligned[f"{offset * tick_us:+.1f}us"] = evaluate(best + offset)[0]

    report = {
        "settings": vars(args),
        "ideal": ideal,
        "hardware": {
            "best_envelope_advance_us": best * tick_us,
            "aligned": hw_best,
            "misaligned_by_one_update": misaligned,
            "alignment_sweep_osb_db": {
                f"{k * tick_us:+.1f}": v[0]["opposite_sideband_suppression_db"]
                for k, v in sorted(sweep.items())},
        },
    }

    prefix = pathlib.Path(args.output_prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    stem = prefix.name
    write_pcm16(prefix.with_name(stem + "-target.wav"), args.sample_rate,
                target, normalize=True)
    write_pcm16(prefix.with_name(stem + "-receiver.wav"), args.sample_rate,
                ideal_received, normalize=True)
    write_pcm16(prefix.with_name(stem + "-hw-receiver.wav"), args.sample_rate,
                hw_received, normalize=True)
    np.savez_compressed(prefix.with_name(stem + "-data.npz"),
                        amplitude=amplitude.astype(np.float32),
                        phase=phase.astype(np.float32),
                        target=target.astype(np.float32))
    with open(prefix.with_name(stem + "-metrics.json"), "w",
              encoding="utf-8") as destination:
        json.dump(report, destination, indent=2)
        destination.write("\n")

    def line(name, m):
        acp = max(m["upper_adjacent_db"], m["lower_adjacent_db"])
        return (f"{name:<10} SI-SDR {m['si_sdr_db']:6.2f} dB  "
                f"OSB {m['opposite_sideband_suppression_db']:5.1f} dB  "
                f"carrier {m['carrier_relative_db']:6.1f} dB  "
                f"ACP {acp:6.1f} dB  far {m['far_out_db']:6.1f} dB  "
                f"wanted {m['wanted_power_percent']:5.1f}%  "
                f"OBW99 {m['occupied_bandwidth_99_hz']:6.0f} Hz")

    print()
    print(line("ideal", ideal))
    print(line("hardware", hw_best) +
          f"  (envelope advance {best * tick_us:+.1f} us)")
    for name, m in misaligned.items():
        print(line(name, m))
    print(f"Wrote {prefix}-{{target,receiver,hw-receiver}}.wav, "
          f"{prefix}-data.npz, {prefix}-metrics.json")


if __name__ == "__main__":
    main()
