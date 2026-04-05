#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  record-macos-system-audio.sh --list
  record-macos-system-audio.sh [OUTPUT.wav] [DURATION_SECONDS]

Environment overrides:
  AUDIO_DEVICE   AVFoundation input name (default: BlackHole 2ch)
  SAMPLE_RATE    Recording sample rate (default: 48000)
  CHANNELS       Recording channels (default: 2)

Without DURATION_SECONDS, press q in ffmpeg to stop recording.
EOF
}

if ! command -v ffmpeg >/dev/null 2>&1; then
    echo "ffmpeg is required: brew install ffmpeg" >&2
    exit 1
fi

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    usage
    exit 0
fi

if [[ "${1:-}" == "--list" ]]; then
    ffmpeg -hide_banner -f avfoundation -list_devices true -i ""
    exit 0
fi

audio_device=${AUDIO_DEVICE:-BlackHole 2ch}
sample_rate=${SAMPLE_RATE:-48000}
channels=${CHANNELS:-2}
output=${1:-polar-ssb-$(date +%Y%m%d-%H%M%S).wav}
duration=${2:-}

duration_args=()
if [[ -n "$duration" ]]; then
    duration_args=(-t "$duration")
fi

echo "Recording '$audio_device' to '$output' at ${sample_rate} Hz / ${channels} ch"
ffmpeg \
    -hide_banner \
    -thread_queue_size 512 \
    -f avfoundation \
    -i ":${audio_device}" \
    "${duration_args[@]}" \
    -ar "$sample_rate" \
    -ac "$channels" \
    -c:a pcm_s24le \
    "$output"

