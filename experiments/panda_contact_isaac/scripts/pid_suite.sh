#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../../.."
S=experiments/panda_contact_isaac/scripts
R=experiments/panda_contact_isaac/runs
for mode_name in position admittance; do
 "$S/isaac-python" "$S/run.py" --inner pid --mode "$mode_name" --out "$R/pid_$mode_name" > "$R/pid_$mode_name.log" 2>&1
 test -f "$R/pid_$mode_name/COMPLETE"
done
"$S/isaac-python" "$S/run.py" --inner pid --mode admittance --contact-gate --out "$R/pid_contact_gate" > "$R/pid_contact_gate.log" 2>&1
test -f "$R/pid_contact_gate/COMPLETE"

"$S/isaac-python" "$S/run.py" --inner pid --mode admittance --contact-gate --speed .0025 --duration 32 --out "$R/pid_gate_slow" > "$R/pid_gate_slow.log" 2>&1
test -f "$R/pid_gate_slow/COMPLETE"
