#!/usr/bin/env python3
"""
Convert C header audio array back to WAV file for debugging.

Usage:
    python3 c_to_wav.py <input_header_file> <output_wav_file> [sample_rate]

Example:
    python3 c_to_wav.py src/audio_data3.h output.wav 32000
"""

import sys
import os
import re

try:
    import numpy as np
    import soundfile as sf
except ImportError:
    print("Error: numpy and soundfile required.")
    print("Install with: pip3 install numpy soundfile")
    sys.exit(1)


def c_header_to_wav(header_file, output_file, sample_rate=32000):
    """Parse C header int16_t array and write WAV file."""

    print(f"Reading C header: {header_file}")

    with open(header_file, 'r') as f:
        content = f.read()

    # Find the array name and data
    # Match: const int16_t <name>[] = { ... };
    array_match = re.search(r'const\s+int16_t\s+(\w+)\s*\[\]\s*=\s*\{(.+?)\};', content, re.DOTALL)
    if not array_match:
        print("Error: Could not find int16_t array in header file")
        sys.exit(1)

    array_name = array_match.group(1)
    array_data = array_match.group(2)

    # Parse comma-separated integers
    samples = []
    for token in array_data.split(','):
        token = token.strip()
        if token and token != '//':
            # Skip comments
            if token.startswith('//'):
                break
            try:
                samples.append(int(token))
            except ValueError:
                pass  # Skip non-integer tokens

    if not samples:
        print("Error: No samples found in header file")
        sys.exit(1)

    print(f"Array name: {array_name}")
    print(f"Samples: {len(samples)}")
    print(f"Sample rate: {sample_rate} Hz")
    print(f"Duration: {len(samples)/sample_rate:.2f} seconds")
    print(f"Min value: {min(samples)}")
    print(f"Max value: {max(samples)}")
    print(f"Mean value: {np.mean(samples):.2f}")

    # Convert to float32 for WAV
    data = np.array(samples, dtype=np.float32) / 32768.0

    # Write WAV
    print(f"Writing WAV: {output_file}")
    sf.write(output_file, data, sample_rate, subtype='PCM_16')

    print(f"Done! {len(samples)} samples written.")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    header_file = sys.argv[1]
    output_file = sys.argv[2]
    sample_rate = int(sys.argv[3]) if len(sys.argv) > 3 else 32000

    if not os.path.exists(header_file):
        print(f"Error: Header file not found: {header_file}")
        sys.exit(1)

    c_header_to_wav(header_file, output_file, sample_rate)
