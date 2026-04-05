#!/usr/bin/env bash
set -euo pipefail

root_dir=$(cd "$(dirname "$0")/.." && pwd)
port=${1:-${POLAR_PORT:-/dev/tty.usbmodem11301}}
image_dir=${2:-$root_dir/build-output}
baud=${ESPTOOL_BAUD:-460800}

for image in bootloader.bin partition-table.bin polar_ssb_si5351_esp32s3.bin; do
    if [[ ! -f "$image_dir/$image" ]]; then
        echo "Missing $image_dir/$image; run make first" >&2
        exit 1
    fi
done

if command -v esptool >/dev/null 2>&1; then
    esptool_command=(esptool)
elif python3 -c 'import esptool' >/dev/null 2>&1; then
    esptool_command=(python3 -m esptool)
else
    echo "Install esptool first: python3 -m pip install esptool" >&2
    exit 1
fi

"${esptool_command[@]}" \
    --chip esp32s3 \
    --port "$port" \
    --baud "$baud" \
    --before default-reset \
    --after hard-reset \
    write-flash \
    --flash-mode dio \
    --flash-freq 80m \
    --flash-size 2MB \
    0x0 "$image_dir/bootloader.bin" \
    0x8000 "$image_dir/partition-table.bin" \
    0x10000 "$image_dir/polar_ssb_si5351_esp32s3.bin"
