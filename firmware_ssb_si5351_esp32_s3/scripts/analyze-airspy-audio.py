#!/usr/bin/env python3
"""Dependency-free timing and level analysis for demodulated Airspy WAV audio."""

import argparse
import array
import math
import os
import wave


def read_mono(path):
    with wave.open(path, "rb") as wav:
        rate = wav.getframerate()
        channels = wav.getnchannels()
        width = wav.getsampwidth()
        frames = wav.getnframes()
        raw = wav.readframes(frames)
    if width != 2:
        raise RuntimeError(f"{path}: expected 16-bit PCM, found {width * 8}-bit")
    values = array.array("h")
    values.frombytes(raw)
    mono = list(values if channels == 1 else values[0::channels])
    identical_channels = True
    if channels > 1:
        left = values[0::channels]
        for channel in range(1, channels):
            if left != values[channel::channels]:
                identical_channels = False
                break
    return rate, channels, mono, identical_channels


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(fraction * len(ordered)))]


def features(rate, samples, frame_ms):
    frame_samples = round(rate * frame_ms / 1000)
    rms = []
    zcr = []
    peak_difference = []
    for start in range(0, len(samples) - frame_samples + 1, frame_samples):
        block = samples[start : start + frame_samples]
        mean = sum(block) / frame_samples
        power = sum((value - mean) ** 2 for value in block) / frame_samples
        rms.append(math.sqrt(power) + 1e-12)
        zcr.append(
            sum((block[index] < 0) != (block[index - 1] < 0)
                for index in range(1, frame_samples)) / frame_samples
        )
        peak_difference.append(
            max(abs(block[index] - block[index - 1])
                for index in range(1, frame_samples))
        )
    log_rms = [20 * math.log10(value / 32768.0) for value in rms]
    return log_rms, zcr, peak_difference


def correlation(first, second):
    count = min(len(first), len(second))
    first = first[:count]
    second = second[:count]
    first_mean = sum(first) / count
    second_mean = sum(second) / count
    first_energy = sum((value - first_mean) ** 2 for value in first)
    second_energy = sum((value - second_mean) ** 2 for value in second)
    if first_energy == 0 or second_energy == 0:
        return 0.0
    cross = sum(
        (first[index] - first_mean) * (second[index] - second_mean)
        for index in range(count)
    )
    return cross / math.sqrt(first_energy * second_energy)


def interpolate(values, position):
    index = int(position)
    if index < 0 or index + 1 >= len(values):
        return None
    fraction = position - index
    return values[index] * (1 - fraction) + values[index + 1] * fraction


def best_repeat(envelope, expected_frames):
    start = max(1, int(expected_frames * 0.95))
    end = min(len(envelope) - 1, int(expected_frames * 1.05))
    compare_frames = min(int(expected_frames * 0.90), len(envelope) // 2)
    best = (-2.0, 0)
    for lag in range(start, end + 1):
        score = correlation(envelope[:compare_frames], envelope[lag : lag + compare_frames])
        if score > best[0]:
            best = (score, lag)
    return best


def best_reference(capture, reference, frame_ms):
    best = (-2.0, 0.0, 0)
    offset_limit = round(500 / frame_ms)
    for ratio_step in range(970, 1031):
        ratio = ratio_step / 1000.0
        for offset in range(-offset_limit, offset_limit + 1):
            candidate = []
            target = []
            for index in range(20, min(len(reference) - 20, 900)):
                value = interpolate(capture, offset + index / ratio)
                if value is not None:
                    candidate.append(value)
                    target.append(reference[index])
            if len(candidate) > 400:
                score = correlation(candidate, target)
                if score > best[0]:
                    best = (score, ratio, offset)
    return best


def describe(path, frame_ms):
    rate, channels, samples, identical_channels = read_mono(path)
    envelope, zcr, peak_difference = features(rate, samples, frame_ms)
    peak = max(abs(value) for value in samples)
    rms = math.sqrt(sum(value * value for value in samples) / len(samples))
    print(f"{path}:")
    print(f"  format: {rate} Hz, {channels} channel(s), 16-bit PCM")
    print(f"  duration: {len(samples) / rate:.6f} s")
    print(f"  channels bit-identical: {'yes' if identical_channels else 'no'}")
    print(f"  peak: {20 * math.log10(peak / 32768):.2f} dBFS")
    print(f"  RMS: {20 * math.log10(rms / 32768):.2f} dBFS")
    print(f"  frame RMS median/p95: {percentile(envelope, .5):.2f} / "
          f"{percentile(envelope, .95):.2f} dBFS")
    print(f"  frame delta p99/max: {percentile(peak_difference, .99)} / "
          f"{max(peak_difference)} samples")
    return rate, samples, envelope


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", nargs="?", default="output.wav")
    parser.add_argument("--original", default="simulate/Original_Kore.wav")
    parser.add_argument("--speech-dsp", default="simulate/speech_dsp_out.wav")
    parser.add_argument("--phase-sim", default="simulate/loopback_out.wav")
    parser.add_argument("--frame-ms", type=int, default=10)
    args = parser.parse_args()

    capture_rate, capture_samples, capture_envelope = describe(
        args.capture, args.frame_ms
    )
    references = []
    for path in (args.original, args.speech_dsp, args.phase_sim):
        if os.path.exists(path):
            references.append((path, describe(path, args.frame_ms)))

    if references:
        original_duration = len(references[0][1][1]) / references[0][1][0]
        expected_frames = round(original_duration * 1000 / args.frame_ms)
        repeat_score, repeat_lag = best_repeat(capture_envelope, expected_frames)
        print("capture repeat:")
        print(f"  lag: {repeat_lag * args.frame_ms / 1000:.3f} s")
        print(f"  envelope correlation: {repeat_score:.6f}")

    for path, (_, _, reference_envelope) in references:
        score, ratio, offset = best_reference(
            capture_envelope, reference_envelope, args.frame_ms
        )
        print(f"capture versus {path}:")
        print(f"  envelope correlation: {score:.4f}")
        print(f"  playback-rate ratio: {ratio:.4f}")
        print(f"  alignment offset: {offset * args.frame_ms / 1000:.3f} s")

    differences = [
        abs(capture_samples[index] - capture_samples[index - 1])
        for index in range(1, len(capture_samples))
    ]
    for threshold in (2000, 4000, 6000):
        count = sum(value >= threshold for value in differences)
        print(
            f"capture |sample delta| >= {threshold}: {count} "
            f"({100 * count / len(differences):.4f}%)"
        )


if __name__ == "__main__":
    main()

