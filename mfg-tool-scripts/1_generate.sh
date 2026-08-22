#!/usr/bin/env bash
# Step 1: Generate manufacturing partition using esp-matter-mfg-tool
set -euo pipefail

cd "$(dirname "$0")"

if ! command -v esp-matter-mfg-tool &>/dev/null; then
    echo "Error: esp-matter-mfg-tool not found in PATH" >&2
    exit 1
fi

esp-matter-mfg-tool \
    --vendor-id 0xFFF1 \
    --product-id 0x8003 \
    --vendor-name Twinsen \
    --product-name "Roller Blinds" \
    --hw-ver 1 \
    --hw-ver-str "01"
