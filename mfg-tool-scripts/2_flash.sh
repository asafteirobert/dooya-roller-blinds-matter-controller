#!/usr/bin/env bash
# Step 3: Flash the manufacturing partition to the device
set -euo pipefail

cd "$(dirname "$0")"

PORT="${1:-${PORT:-/dev/ttyACM0}}"

if ! command -v esptool.py &>/dev/null; then
    echo "Error: esptool.py not found in PATH" >&2
    exit 1
fi

PARTITION_BIN=$(ls out/fff1_8003/*/*-partition.bin 2>/dev/null | head -1)
if [[ -z "$PARTITION_BIN" ]]; then
    echo "Error: No partition.bin found under out/fff1_8003/. Run step 1 first." >&2
    exit 1
fi

echo "Flashing $PARTITION_BIN to $PORT..."
esptool.py -p "$PORT" write_flash 0x10000 "$PARTITION_BIN"
