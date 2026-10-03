#!/usr/bin/env bash
# Development validation only; never substitutes for qualified paper repeats.
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$task_root"
scene="${1:-}"
results="runs/final_validation_20260925"
flags=(--development --speedup 2)
case "$scene" in
  straight)
    config="runs/fixedwing_upgrade_20260925/tasks_v3/rotor_straight_12.yaml"
    flags+=(--copter-yaw-rate --copter-synchronous)
    ;;
  s1)
    config="$results/s1_turns_reserve_0.5.json"
    flags+=(--copter-yaw-rate --copter-synchronous)
    ;;
  s2)
    config="$results/s2_transition_160.json"
    flags+=(--plane-progress-gain 0.05 --plane-command-accel-scale 0.5)
    ;;
  s3)
    config="$results/s3_left_wing_positive_smooth.json"
    flags+=(--copter-yaw-rate --copter-synchronous --plane-progress-gain 0.05
      --rotor-precheck "$results/rotor_sync_12_01"
      --rotor-precheck runs/execution_dynamics_20260925/rotor_sync_14_01)
    ;;
  *) echo 'Usage: bash scripts/reproduce_feedback_validation.sh straight|s1|s2|s3 [new-output-directory]' >&2; exit 2;;
esac
output="${2:-runs/feedback-retest-${scene}-$(date +%Y%m%d-%H%M%S)}"
if [[ -e "$output" ]]; then
  echo "Refusing to overwrite or re-audit an existing run: $output" >&2
  exit 2
fi
port="$(python3 - <<'PY'
import socket
for base in range(32000, 39000, 100):
    sockets=[]
    try:
        for member in range(3):
            for offset in (0,2,3):
                s=socket.socket(); sockets.append(s)
                s.bind(('127.0.0.1',base+10*member+offset))
        print(base)
        break
    except OSError:
        pass
    finally:
        for s in sockets: s.close()
else:
    raise SystemExit('No free local SITL port group')
PY
)"
run_status=0
bash scripts/experiment_env.sh python3 scripts/run_paper_sitl.py \
  --config "$config" --output "$output" --base-port "$port" "${flags[@]}" || run_status=$?
if [[ -f "$output/flight_status.json" ]]; then
  bash scripts/experiment_env.sh python3 scripts/audit_tracking_run.py "$output"
  if [[ "$scene" != s2 ]]; then
    bash scripts/experiment_env.sh python3 scripts/audit_measured_feedback.py "$output" || run_status=1
  fi
  bash scripts/experiment_env.sh python3 scripts/plot_paper_run.py "$output"
  # A completed flight is not a passed constraint audit.
  python3 - "$output/acceptance.json" <<'PY' || run_status=1
import json,sys
from pathlib import Path
path=Path(sys.argv[1]); a=json.loads(path.read_text())
print('Acceptance:',path.resolve())
print('Tracking:',a['tracking_qualified'],'Measured constraints:',a['measured_constraints_passed'],
      'Formal qualification:',a['formal_qualified'])
raise SystemExit(0 if a['tracking_qualified'] and a['measured_constraints_passed'] else 1)
PY
fi
exit "$run_status"
