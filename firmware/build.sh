#!/bin/sh
# Build the RL-only firmware and export it with a manifest:  sh build.sh   ->  dist/{firmware.bin,manifest.json}
set -eu
cd "$(dirname "$0")"
if [ ! -x .venv/bin/python ]; then
    python3 -m venv .venv
    .venv/bin/python -m pip install -r requirements.txt
fi
.venv/bin/python -m platformio run -e rider2-rl-only
.venv/bin/python tools/export_firmware.py --environment rider2-rl-only --output-dir dist
