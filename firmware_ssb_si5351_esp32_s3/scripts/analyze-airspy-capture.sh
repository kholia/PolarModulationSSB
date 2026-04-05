#!/usr/bin/env bash
set -euo pipefail

root_dir=$(cd "$(dirname "$0")/.." && pwd)
capture=${1:-output.wav}
report_dir=${2:-analysis-output}

if [[ ! -f "$capture" ]]; then
    echo "Capture not found: $capture" >&2
    exit 1
fi

for command_name in ffmpeg ffprobe sox python3; do
    if ! command -v "$command_name" >/dev/null 2>&1; then
        echo "$command_name is required" >&2
        exit 1
    fi
done

mkdir -p "$report_dir"

ffprobe -v error \
    -show_entries format=duration,size,bit_rate:stream=index,codec_name,sample_rate,channels,channel_layout,bits_per_sample \
    -of default=noprint_wrappers=1 \
    "$capture" >"$report_dir/metadata.txt"

ffmpeg -hide_banner -i "$capture" \
    -af "astats=metadata=1:reset=0" \
    -f null - >"$report_dir/levels.txt" 2>&1

ffmpeg -y -hide_banner -i "$capture" \
    -lavfi "pan=mono|c0=c0,showspectrumpic=s=1600x900:legend=1:color=viridis:scale=log:fscale=lin:gain=4" \
    "$report_dir/spectrogram.png" >"$report_dir/spectrogram.log" 2>&1

python3 "$root_dir/scripts/analyze-airspy-audio.py" "$capture" \
    --original "$root_dir/simulate/Original_Kore.wav" \
    --speech-dsp "$root_dir/simulate/speech_dsp_out.wav" \
    --phase-sim "$root_dir/simulate/loopback_out.wav" \
    | tee "$report_dir/timing-and-levels.txt"

band_rms() {
    local input_file=$1
    local band=$2
    if [[ "$band" == "0-300" ]]; then
        sox "$input_file" -n remix 1 sinc -300 stats 2>&1 \
            | awk '/RMS lev dB/ {print $NF}'
    else
        sox "$input_file" -n remix 1 sinc "$band" stats 2>&1 \
            | awk '/RMS lev dB/ {print $NF}'
    fi
}

{
    printf "file\tband_hz\trms_dbfs\n"
    for input_file in \
        "$capture" \
        "$root_dir/simulate/Original_Kore.wav" \
        "$root_dir/simulate/speech_dsp_out.wav" \
        "$root_dir/simulate/loopback_out.wav"; do
        [[ -f "$input_file" ]] || continue
        for band in 0-300 300-1000 1000-2000 2000-3000 3000-4200 4200-8000; do
            printf "%s\t%s\t%s\n" "$input_file" "$band" "$(band_rms "$input_file" "$band")"
        done
    done
} | tee "$report_dir/band-levels.tsv"

echo "Analysis written to $report_dir"

