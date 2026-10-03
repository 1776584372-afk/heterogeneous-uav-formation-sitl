#!/usr/bin/env bash
set -euo pipefail
PAPER_DEMO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$PAPER_DEMO_ROOT/scripts/experiment_env.sh" python3 "$PAPER_DEMO_ROOT/scripts/paper_demo.py" "$@"
