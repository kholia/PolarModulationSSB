#!/usr/bin/env bash
# Compare classic EER, phase-only and the joint optimizer across drain
# envelope bandwidths. Prints the ideal and hardware-model lines per run;
# full metrics land in <outdir>/<run>-metrics.json.
#
#   simulate/run_polar_sweep.sh [source.wav] [outdir] [extra optimizer args]
set -euo pipefail

script_dir=$(cd "$(dirname "$0")" && pwd)
root_dir=$(cd "$script_dir/.." && pwd)
input=${1:-$root_dir/ESP32-S3-Si5351-Sample.wav}
outdir=${2:-$script_dir/polar-sweep}
if (( $# > 0 )); then shift; fi
if (( $# > 0 )); then shift; fi
mkdir -p "$outdir"

run() {
    local name=$1
    shift
    echo "== $name"
    uv run --quiet "$script_dir/optimize_polar_joint.py" "$input" \
        --output-prefix "$outdir/$name" --report-every 100000 "$@" \
        | grep -E '^(ideal|hardware|[-+][0-9])'
}

run phase-only --mode phase-only --iterations 150 "$@"
for bw in 500 1000 2000 3000 6000; do
    run "classic-$bw" --mode classic --envelope-bandwidth "$bw" "$@"
done
for bw in 500 1000 2000 3000; do
    for weight in 3 10; do
        run "joint-$bw-w$weight" --mode joint --envelope-bandwidth "$bw" \
            --iterations 1000 --phase-rate 0.05 --amplitude-rate 0.005 \
            --stop-weight "$weight" --far-weight "$weight" "$@"
    done
done
