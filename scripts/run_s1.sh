#!/usr/bin/env bash
set -e
EXPERIMENT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$EXPERIMENT_ROOT/scripts/experiment_env.sh" python3 \
    "$EXPERIMENT_ROOT/scripts/run_experiment.py" "$@"
