#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../../.."
SIM_RUN=experiments/panda_contact_isaac/scripts/isaac-python
SCRIPT_RUN=experiments/panda_contact_isaac/scripts/run.py
RUN_DIR=experiments/panda_contact_isaac/runs
run_case() {
 local case_name="$1"
 shift
 "$SIM_RUN" "$SCRIPT_RUN" --inner compensated "$@" --out "$RUN_DIR/$case_name" > "$RUN_DIR/$case_name.log" 2>&1
 test -f "$RUN_DIR/$case_name/COMPLETE"
}
run_case position_final --mode position --record
run_case admittance_final --mode admittance --record
run_case admittance_slow --mode admittance --speed .0025 --duration 32
run_case admittance_B20 --mode admittance --damping 20
run_case admittance_limited --mode admittance --offset-limit .03
.venvs/qwen-scene/bin/python experiments/panda_contact_isaac/scripts/analyze.py > "$RUN_DIR/analysis.log"
