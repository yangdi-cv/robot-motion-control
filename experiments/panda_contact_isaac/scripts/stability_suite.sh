#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../../.."
S=experiments/panda_contact_isaac/scripts
R=experiments/panda_contact_isaac/runs
run_case() {
 local name="$1"
 shift
 "$S/isaac-python" "$S/run.py" --inner pid --mode admittance --contact-gate "$@" --out "$R/$name" > "$R/$name.log" 2>&1
 test -f "$R/$name/COMPLETE"
}
run_case stability_freeze --freeze-on-contact
run_case stability_freeze_ema --freeze-on-contact --force-feedback ema
run_case stability_freeze_last --freeze-on-contact --force-feedback last
run_case stability_freeze_5ms --freeze-on-contact --control-dt .005
run_case stability_freeze_20ms --freeze-on-contact --control-dt .02
