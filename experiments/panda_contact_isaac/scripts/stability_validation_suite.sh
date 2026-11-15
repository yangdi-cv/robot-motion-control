#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../../.."
S=experiments/panda_contact_isaac/scripts
R=experiments/panda_contact_isaac/runs
for name in repeat higher lower; do
 shift_value=0
 if [ "$name" = higher ]; then shift_value=.001; fi
 if [ "$name" = lower ]; then shift_value=-.001; fi
 "$S/isaac-python" "$S/run.py" --inner pid --mode admittance --contact-gate --freeze-on-contact --ik-seed differential --force-feedback ema --plane-height-shift "$shift_value" --out "$R/stability_validation_$name" > "$R/stability_validation_$name.log" 2>&1
 test -f "$R/stability_validation_$name/COMPLETE"
done
