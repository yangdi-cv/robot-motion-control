#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../../.."
S=experiments/panda_contact_isaac/scripts
R=experiments/panda_contact_isaac/runs
for name in moving frozen; do
 args=()
 if [ "$name" = frozen ]; then args+=(--freeze-on-contact); fi
 "$S/isaac-python" "$S/run.py" --inner pid --mode admittance --contact-gate --ik-seed differential "${args[@]}" --out "$R/stability_differential_$name" > "$R/stability_differential_$name.log" 2>&1
 test -f "$R/stability_differential_$name/COMPLETE"
done
