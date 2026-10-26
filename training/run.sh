#!/bin/sh
set -eu
cd "$(dirname "$0")"
task_name="${1:-view}"
if [ "$#" -gt 0 ]; then shift; fi
case "$task_name" in
    setup) exec sh setup.sh "$@" ;;
    view) exec .venv/bin/python viewer2.py "$@" ;;
    evaluate) exec .venv/bin/python evaluate2.py --policy native artifacts/balance-v6/policy_weights.h "$@" ;;
    train) exec .venv/bin/python train2.py "$@" ;;
    verify) exec .venv/bin/python verify_export2.py artifacts/balance-v6 "$@" ;;
    margin) exec .venv/bin/python tools/loop_margin.py artifacts/balance-v6/actor.json "$@" ;;
    frf) exec .venv/bin/python tools/sim_frf_check.py "$@" ;;
    *) echo 'Usage: sh run.sh setup|view|evaluate|train|verify|margin|frf   (validate/video were tied to the contract-5 simulator and were retired, see REBUILD.md)' >&2; exit 2 ;;
esac
