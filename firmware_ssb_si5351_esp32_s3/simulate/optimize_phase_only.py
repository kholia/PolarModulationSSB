#!/usr/bin/env python3
"""Iteratively synthesize a constant-envelope phase sequence for SSB audio.

This is an offline upper-bound experiment. It alternates between frequency-
domain constraints (match wanted USB audio, reject carrier/opposite sideband)
and the time-domain constant-envelope constraint |z[n]| = 1. Energy that
cannot satisfy both constraints is allowed to move into a configurable dump
band above the receiver passband.
"""

import argparse
import json
import math
import pathlib
import wave

import numpy as np


def read_pcm_wav(path):
    with wave.open(str(path), "rb") as source:
        rate = source.getframerate()
        channels = source.getnchannels()
        width = source.getsampwidth()
        frames = source.getnframes()
        raw = source.readframes(frames)
    if width == 2:
        values = np.frombuffer(raw, dtype="<i2").astype(np.float64) / 32768.0
    elif width == 3:
        bytes_ = np.frombuffer(raw, dtype=np.uint8).reshape(-1, 3)
        values = (bytes_[:, 0].astype(np.int32) |
                  (bytes_[:, 1].astype(np.int32) << 8) |
                  (bytes_[:, 2].astype(np.int32) << 16))
        values = np.where(values & 0x800000, values - 0x1000000, values)
        values = values.astype(np.float64) / 8388608.0
    elif width == 4:
        values = np.frombuffer(raw, dtype="<i4").astype(np.float64) / 2147483648.0
    else:
        raise RuntimeError(f"unsupported PCM width: {width * 8} bits")
    values = values.reshape(-1, channels).mean(axis=1)
    return rate, values


def write_pcm16(path, rate, samples, normalize=False):
    samples = np.asarray(samples, dtype=np.float64)
    if normalize:
        peak = np.max(np.abs(samples)) if samples.size else 0.0
        if peak > 0:
            samples = samples * (0.95 / peak)
    encoded = np.clip(np.rint(samples * 32767.0), -32768, 32767).astype("<i2")
    with wave.open(str(path), "wb") as destination:
        destination.setnchannels(1)
        destination.setsampwidth(2)
        destination.setframerate(rate)
        destination.writeframes(encoded.tobytes())


def resample(samples, source_rate, destination_rate):
    if source_rate == destination_rate:
        return samples.copy()
    if source_rate > destination_rate:
        # Anti-alias before interpolation. The wanted audio is below 3 kHz,
        # but this prevents ultrasonic capture noise folding into that band.
        taps = 255
        cutoff = 0.45 * destination_rate / source_rate
        index = np.arange(taps) - (taps - 1) / 2
        kernel = 2 * cutoff * np.sinc(2 * cutoff * index)
        kernel *= np.blackman(taps)
        kernel /= np.sum(kernel)
        samples = np.convolve(samples, kernel, mode="same")
    duration = len(samples) / source_rate
    output_count = int(round(duration * destination_rate))
    source_positions = np.arange(output_count) * source_rate / destination_rate
    return np.interp(source_positions, np.arange(len(samples)), samples)


def next_power_of_two(value):
    return 1 << max(1, int(value - 1).bit_length())


def wrap_phase(value):
    return (value + np.pi) % (2 * np.pi) - np.pi


def project_phase_slew(phase, maximum_step, passes=24):
    if maximum_step <= 0:
        return phase
    # Alternating projections over the even and odd adjacent-pair constraint
    # sets. Unlike a forward cumulative clip, this distributes correction on
    # both sides of a violation and does not turn one early jump into a phase
    # error across the remainder of the recording.
    projected = np.unwrap(phase).copy()
    for _ in range(passes):
        for start in (0, 1):
            left = projected[start:-1:2]
            right = projected[start + 1::2]
            difference = right - left
            excess = np.maximum(np.abs(difference) - maximum_step, 0.0)
            correction = 0.5 * np.sign(difference) * excess
            left += correction
            right -= correction
    return projected


def correlation(first, second):
    first = first - np.mean(first)
    second = second - np.mean(second)
    denominator = np.linalg.norm(first) * np.linalg.norm(second)
    return float(np.dot(first, second) / denominator) if denominator else 0.0


def si_sdr(reference, estimate):
    reference = reference - np.mean(reference)
    estimate = estimate - np.mean(estimate)
    scale = np.dot(estimate, reference) / (np.dot(reference, reference) + 1e-30)
    wanted = scale * reference
    error = estimate - wanted
    return float(10 * np.log10(np.dot(wanted, wanted) /
                               (np.dot(error, error) + 1e-30)))


def measurements(z, desired_spectrum, wanted, unwanted, dump, sample_count):
    spectrum = np.fft.fft(z)
    received_complex = np.fft.ifft(spectrum * wanted)
    target_complex = np.fft.ifft(desired_spectrum)
    received = np.real(received_complex[:sample_count])
    target = np.real(target_complex[:sample_count])
    wanted_energy = np.sum(np.abs(spectrum[wanted]) ** 2)
    unwanted_energy = np.sum(np.abs(spectrum[unwanted]) ** 2)
    total_energy = np.sum(np.abs(spectrum) ** 2)
    wanted_error = np.linalg.norm((spectrum - desired_spectrum)[wanted])
    wanted_reference = np.linalg.norm(desired_spectrum[wanted])
    steps = wrap_phase(np.diff(np.angle(z)))
    return {
        "correlation": correlation(target, received),
        "si_sdr_db": si_sdr(target, received),
        "wanted_error_db": float(20 * np.log10(
            wanted_error / (wanted_reference + 1e-30) + 1e-30)),
        "opposite_sideband_suppression_db": float(10 * np.log10(
            wanted_energy / (unwanted_energy + 1e-30))),
        "dump_band_energy_percent": float(
            100 * np.sum(np.abs(spectrum[dump]) ** 2) / (total_energy + 1e-30)),
        "carrier_relative_db": float(20 * np.log10(
            np.abs(spectrum[0]) / (math.sqrt(wanted_energy) + 1e-30) + 1e-30)),
        "phase_step_rms": float(np.sqrt(np.mean(steps * steps))),
        "phase_step_p99": float(np.percentile(np.abs(steps), 99)),
        "phase_step_max": float(np.max(np.abs(steps))),
    }, target, received


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="PCM WAV containing the source voice")
    parser.add_argument("--output-prefix", default="simulate/phase-optimized")
    parser.add_argument("--sample-rate", type=int, default=16000)
    parser.add_argument("--duration", type=float, default=10.0,
                        help="seconds to optimize; 0 uses the entire file")
    parser.add_argument("--iterations", type=int, default=150)
    parser.add_argument("--low-hz", type=float, default=300.0)
    parser.add_argument("--high-hz", type=float, default=3000.0)
    parser.add_argument("--dump-hz", type=float, default=4000.0,
                        help="phase-noise dump band begins at this frequency")
    parser.add_argument("--target-peak", type=float, default=0.70,
                        help="peak desired analytic envelope, below unity")
    parser.add_argument("--spectral-relaxation", type=float, default=0.65)
    parser.add_argument("--stopband-retention", type=float, default=0.02,
                        help="0 fully zeros protected bins each iteration")
    parser.add_argument("--max-phase-step", type=float, default=0.0,
                        help="radians/sample; 0 finds the unconstrained ceiling")
    parser.add_argument("--report-every", type=int, default=10)
    args = parser.parse_args()

    if not 0 < args.target_peak <= 1:
        parser.error("--target-peak must be in (0, 1]")
    if not 0 < args.spectral_relaxation <= 1:
        parser.error("--spectral-relaxation must be in (0, 1]")
    if not 0 <= args.stopband_retention <= 1:
        parser.error("--stopband-retention must be in [0, 1]")
    nyquist = args.sample_rate / 2
    if not 0 < args.low_hz < args.high_hz < args.dump_hz < nyquist:
        parser.error("require 0 < low-hz < high-hz < dump-hz < Nyquist")

    source_rate, source = read_pcm_wav(args.input)
    if args.duration > 0:
        source = source[:round(args.duration * source_rate)]
    source = resample(source, source_rate, args.sample_rate)
    source -= np.mean(source)
    sample_count = len(source)
    fft_size = next_power_of_two(sample_count)
    padded = np.zeros(fft_size, dtype=np.float64)
    padded[:sample_count] = source
    edge = min(round(0.01 * args.sample_rate), sample_count // 2)
    if edge:
        fade = np.sin(np.linspace(0, np.pi / 2, edge)) ** 2
        padded[:edge] *= fade
        padded[sample_count - edge:sample_count] *= fade[::-1]

    frequencies = np.fft.fftfreq(fft_size, 1 / args.sample_rate)
    wanted = (frequencies >= args.low_hz) & (frequencies <= args.high_hz)
    unwanted = ((frequencies <= -args.low_hz) &
                (frequencies >= -args.high_hz))
    dump = np.abs(frequencies) >= args.dump_hz
    protected = ~(wanted | dump)

    real_spectrum = np.fft.fft(padded)
    desired_spectrum = np.zeros(fft_size, dtype=np.complex128)
    desired_spectrum[wanted] = 2.0 * real_spectrum[wanted]
    desired_time = np.fft.ifft(desired_spectrum)
    desired_peak = np.max(np.abs(desired_time))
    if desired_peak == 0:
        raise SystemExit("source has no energy in the requested speech band")
    desired_spectrum *= args.target_peak / desired_peak
    desired_time = np.fft.ifft(desired_spectrum)

    # The legacy phase-only method is the natural initialization. Absolute
    # phase during a true silence is arbitrary, so retain the previous value
    # there instead of seeding atan2 noise.
    initial_phase = np.angle(desired_time)
    envelope = np.abs(desired_time)
    quiet = envelope < args.target_peak * 1e-4
    for index in np.flatnonzero(quiet):
        if index:
            initial_phase[index] = initial_phase[index - 1]
    z = np.exp(1j * initial_phase)
    baseline = z.copy()

    print(f"input={args.input} source_rate={source_rate} target_rate={args.sample_rate}")
    print(f"samples={sample_count} fft_size={fft_size} iterations={args.iterations}")
    print(f"wanted={args.low_hz:.0f}-{args.high_hz:.0f} Hz "
          f"dump=|f|>={args.dump_hz:.0f} Hz max_step={args.max_phase_step}")

    for iteration in range(1, args.iterations + 1):
        spectrum = np.fft.fft(z)
        spectrum[wanted] = (
            (1 - args.spectral_relaxation) * spectrum[wanted] +
            args.spectral_relaxation * desired_spectrum[wanted]
        )
        spectrum[protected] *= args.stopband_retention
        candidate = np.fft.ifft(spectrum)
        phase = np.angle(candidate)
        phase = project_phase_slew(phase, args.max_phase_step)
        z = np.exp(1j * phase)
        if iteration == 1 or iteration % args.report_every == 0 or iteration == args.iterations:
            metrics, _, _ = measurements(
                z, desired_spectrum, wanted, unwanted, dump, sample_count
            )
            print(f"iteration={iteration:4d} corr={metrics['correlation']:.6f} "
                  f"SI-SDR={metrics['si_sdr_db']:+.2f} dB "
                  f"OSB={metrics['opposite_sideband_suppression_db']:.1f} dB "
                  f"error={metrics['wanted_error_db']:+.2f} dB")

    baseline_metrics, target, baseline_received = measurements(
        baseline, desired_spectrum, wanted, unwanted, dump, sample_count
    )
    optimized_metrics, _, optimized_received = measurements(
        z, desired_spectrum, wanted, unwanted, dump, sample_count
    )
    report = {
        "settings": vars(args),
        "source_rate": source_rate,
        "sample_count": sample_count,
        "fft_size": fft_size,
        "baseline": baseline_metrics,
        "optimized": optimized_metrics,
    }

    prefix = pathlib.Path(args.output_prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    write_pcm16(prefix.with_name(prefix.name + "-target.wav"),
                args.sample_rate, target, normalize=True)
    write_pcm16(prefix.with_name(prefix.name + "-baseline.wav"),
                args.sample_rate, baseline_received, normalize=True)
    write_pcm16(prefix.with_name(prefix.name + "-receiver.wav"),
                args.sample_rate, optimized_received, normalize=True)
    np.savez_compressed(
        prefix.with_name(prefix.name + "-data.npz"),
        phase=np.angle(z[:sample_count]).astype(np.float32),
        phase_steps=wrap_phase(np.diff(np.angle(z[:sample_count]))).astype(np.float32),
        target=target.astype(np.float32),
        receiver=optimized_received.astype(np.float32),
    )
    with open(prefix.with_name(prefix.name + "-metrics.json"), "w",
              encoding="utf-8") as destination:
        json.dump(report, destination, indent=2)
        destination.write("\n")

    print("\nBaseline:")
    print(json.dumps(baseline_metrics, indent=2))
    print("Optimized:")
    print(json.dumps(optimized_metrics, indent=2))
    print(f"Wrote {prefix}-target.wav, {prefix}-baseline.wav, "
          f"{prefix}-receiver.wav, {prefix}-data.npz and {prefix}-metrics.json")


if __name__ == "__main__":
    main()
