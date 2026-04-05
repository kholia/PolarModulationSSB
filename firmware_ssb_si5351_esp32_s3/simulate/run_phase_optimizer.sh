#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd "$(dirname "$0")" && pwd)
root_dir=$(cd "$script_dir/.." && pwd)
input=${1:-$root_dir/ESP32-S3-Si5351-Sample.wav}
prefix=${2:-$script_dir/phase-optimized}
if (( $# > 0 )); then shift; fi
if (( $# > 0 )); then shift; fi

exec python3 "$script_dir/optimize_phase_only.py" \
    "$input" \
    --output-prefix "$prefix" \
    "$@"
