#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../../.."
S=experiments/panda_contact_isaac/scripts
R=experiments/panda_contact_isaac/runs
run_case() {
 local name="$1"
 shift
 "$S/isaac-python" "$S/run.py" --inner pid --mode admittance --contact-gate --freeze-on-contact --ik-seed differential "$@" --out "$R/$name" > "$R/$name.log" 2>&1
 test -f "$R/$name/COMPLETE"
}
run_case stability_differential_ema --force-feedback ema
run_case stability_differential_5ms --control-dt .005
"$S/isaac-python" "$S/run.py" --inner pid --mode admittance --contact-gate --freeze-on-contact --ik-seed previous --ik-tolerance .000001 --out "$R/stability_ik_guard" > "$R/stability_ik_guard.log" 2>&1
test -f "$R/stability_ik_guard/FAILED.json"
test ! -f "$R/stability_ik_guard/COMPLETE"
