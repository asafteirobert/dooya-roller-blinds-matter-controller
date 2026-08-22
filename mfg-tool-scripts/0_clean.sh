#!/usr/bin/env bash
# Step 0: Remove previous manufacturing tool outputs
set -euo pipefail

cd "$(dirname "$0")"

if [[ ! -d out ]]; then
    echo "Nothing to clean."
    exit 0
fi

rm -rf out
echo "Cleaned out/"
