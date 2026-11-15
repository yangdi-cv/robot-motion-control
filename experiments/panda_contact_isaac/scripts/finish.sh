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
run_case admittance_B80 --mode admittance --damping 80
run_case admittance_repeat --mode admittance
run_case admittance_contact_gate --mode admittance --contact-gate
run_case admittance_force_ramp --mode admittance --contact-gate --force-ramp .5
.venvs/qwen-scene/bin/python experiments/panda_contact_isaac/scripts/analyze.py > "$RUN_DIR/analysis.log"
