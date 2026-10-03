#!/usr/bin/env bash
# Source this file, or use it as a wrapper: bash scripts/experiment_env.sh COMMAND ...
EXPERIMENT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONNOUSERSITE=1
export PYTHONDONTWRITEBYTECODE=1
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
source /opt/ros/humble/setup.bash
if [[ -f "$HOME/ros2_mavros_ws/install/setup.bash" ]]; then
    source "$HOME/ros2_mavros_ws/install/setup.bash"
fi
source "$EXPERIMENT_ROOT/bamboo_src/install/setup.bash"
export PYTHONPATH="$EXPERIMENT_ROOT:$EXPERIMENT_ROOT/.paper_dependencies${PYTHONPATH:+:$PYTHONPATH}"
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    exec "$@"
fi
