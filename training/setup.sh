#!/bin/sh
set -eu
cd "$(dirname "$0")"
if [ ! -x .venv/bin/python ]; then python3 -m venv .venv; fi
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -c 'import mujoco,torch; print("MuJoCo",mujoco.__version__,"PyTorch",torch.__version__)'
