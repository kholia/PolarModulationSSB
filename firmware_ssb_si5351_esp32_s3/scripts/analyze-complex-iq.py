#!/usr/bin/env python3
"""Measure a little-endian interleaved complex-float32 SDR IQ recording."""

import argparse
import json
import pathlib

import numpy as np


def db(value):
    return float(10 * np.log10(value + 1e-300))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", nargs="?", default="0.raw")
    parser.add_argument("--sample-rate", type=float, default=768000.0)
    parser.add_argument("--output-dir", default="iq-analysis")
    parser.add_argument("--fft-size", type=int, default=262144)
    parser.add_argument("--segments", type=int, default=48)
    parser.add_argument("--carrier-search-hz", type=float, default=100.0)
    parser.add_argument("--wanted-low-hz", type=float, default=300.0)
    parser.add_argument("--wanted-high-hz", type=float, default=3000.0)
    parser.add_argument("--dump-low-hz", type=float, default=4500.0)
    parser.add_argument("--dump-high-hz", type=float, default=8000.0)
    parser.add_argument("--plot", action="store_true",
                        help="also create iq-spectrum.png with matplotlib")
    args = parser.parse_args()

    raw = np.memmap(args.input, dtype="<f4", mode="r")
    if len(raw) % 2:
        raw = raw[:-1]
    iq = raw.reshape(-1, 2)
    if len(iq) < args.fft_size:
        parser.error("recording is shorter than one FFT")
    if not np.isfinite(iq[:min(len(iq), 1_000_000)]).all():
        parser.error("input does not look like finite complex float32 IQ")

    window = np.hanning(args.fft_size)
    window_power = np.sum(window * window)
    spectrum_power = np.zeros(args.fft_size)
    segment_count = min(args.segments, max(1, len(iq) // args.fft_size))
    starts = np.linspace(0, len(iq) - args.fft_size, segment_count, dtype=int)
    for start in starts:
        block = iq[start:start + args.fft_size]
        signal = block[:, 0].astype(np.float64) + 1j * block[:, 1].astype(np.float64)
        spectrum = np.fft.fftshift(np.fft.fft(signal * window))
        spectrum_power += np.abs(spectrum) ** 2 / window_power
    spectrum_power /= segment_count
    frequencies = np.fft.fftshift(
        np.fft.fftfreq(args.fft_size, 1 / args.sample_rate)
    )

    carrier_search = np.abs(frequencies) <= args.carrier_search_hz
    carrier_index = np.flatnonzero(carrier_search)[
        np.argmax(spectrum_power[carrier_search])
    ]
    carrier_hz = frequencies[carrier_index]
    relative_frequency = frequencies - carrier_hz

    def energy(low, high):
        mask = ((relative_frequency >= low) &
                (relative_frequency < high))
        return float(np.sum(spectrum_power[mask]))

    wanted = energy(args.wanted_low_hz, args.wanted_high_hz)
    unwanted = energy(-args.wanted_high_hz, -args.wanted_low_hz)
    carrier = energy(-25, 25)
    transition = energy(args.wanted_high_hz, args.dump_low_hz) + \
        energy(-args.dump_low_hz, -args.wanted_high_hz)
    dump = energy(args.dump_low_hz, args.dump_high_hz) + \
        energy(-args.dump_high_hz, -args.dump_low_hz)

    # Estimate a local per-bin floor just beyond the dump band, then subtract
    # it before occupied-bandwidth integration.
    obw_limit = 12000.0
    noise_mask = ((np.abs(relative_frequency) >= args.dump_high_hz) &
                  (np.abs(relative_frequency) < obw_limit))
    noise_per_bin = float(np.median(spectrum_power[noise_mask]))
    signal_power = np.maximum(spectrum_power - noise_per_bin, 0.0)
    obw_mask = np.abs(relative_frequency) < obw_limit
    obw_frequencies = relative_frequency[obw_mask]
    obw_power = signal_power[obw_mask]

    def minimum_obw(fraction):
        cumulative = np.cumsum(obw_power)
        total = cumulative[-1]
        best_width = float("inf")
        best_low = best_high = 0.0
        for left in range(len(obw_power)):
            before = cumulative[left - 1] if left else 0.0
            right = int(np.searchsorted(cumulative, before + fraction * total))
            if right >= len(obw_frequencies):
                break
            width = obw_frequencies[right] - obw_frequencies[left]
            if width < best_width:
                best_width = width
                best_low = obw_frequencies[left]
                best_high = obw_frequencies[right]
        return {
            "width_hz": float(best_width),
            "low_hz": float(best_low),
            "high_hz": float(best_high),
        }

    image_levels = {}
    for image_hz in range(-64000, 64001, 16000):
        image_energy = energy(image_hz - 4000, image_hz + 4000)
        image_levels[str(image_hz)] = db(image_energy / wanted)

    report = {
        "input": args.input,
        "format": "interleaved little-endian complex float32",
        "sample_rate_hz": args.sample_rate,
        "complex_samples": int(len(iq)),
        "duration_seconds": len(iq) / args.sample_rate,
        "fft_size": args.fft_size,
        "resolution_hz": args.sample_rate / args.fft_size,
        "averaged_segments": segment_count,
        "carrier_offset_hz": float(carrier_hz),
        "opposite_sideband_suppression_db": db(wanted / unwanted),
        "carrier_relative_to_wanted_db": db(carrier / wanted),
        "transition_relative_to_wanted_db": db(transition / wanted),
        "dump_relative_to_wanted_db": db(dump / wanted),
        "obw_90": minimum_obw(0.90),
        "obw_95": minimum_obw(0.95),
        "obw_99": minimum_obw(0.99),
        "update_image_energy_relative_to_wanted_db": image_levels,
    }

    output_dir = pathlib.Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "iq-metrics.json").open("w", encoding="utf-8") as output:
        json.dump(report, output, indent=2)
        output.write("\n")

    plot_mask = np.abs(relative_frequency) <= 40000
    plot_frequency = relative_frequency[plot_mask]
    plot_db = 10 * np.log10(spectrum_power[plot_mask] + 1e-300)
    np.savetxt(
        output_dir / "iq-spectrum.tsv",
        np.column_stack((plot_frequency, plot_db - np.max(plot_db))),
        delimiter="\t",
        header="frequency_relative_to_carrier_hz\tpsd_relative_db",
        comments="",
        fmt="%.6f",
    )
    if args.plot:
        try:
            import matplotlib.pyplot as plt
            figure, axis = plt.subplots(figsize=(14, 7))
            axis.plot(plot_frequency / 1000, plot_db - np.max(plot_db), linewidth=0.7)
            axis.set_xlim(-40, 40)
            axis.set_ylim(-80, 3)
            axis.set_xlabel("Frequency relative to carrier (kHz)")
            axis.set_ylabel("PSD relative to peak (dB)")
            axis.grid(True, alpha=0.25)
            figure.tight_layout()
            figure.savefig(output_dir / "iq-spectrum.png", dpi=160)
            plt.close(figure)
        except ImportError:
            print("matplotlib is unavailable; skipping iq-spectrum.png")

    print(json.dumps(report, indent=2))
    print(f"Analysis written to {output_dir}")


if __name__ == "__main__":
    main()
