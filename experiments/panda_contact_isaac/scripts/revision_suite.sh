#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../../.."
S=experiments/panda_contact_isaac/scripts
R=experiments/panda_contact_isaac/runs
run_case() {
 local name="$1"
 shift
 "$S/isaac-python" "$S/run.py" "$@" --out "$R/$name" > "$R/$name.log" 2>&1
 test -f "$R/$name/COMPLETE"
}
for case_name in oracle delayed_naive delayed_timestamp jitter_raw jitter_limited outage_stale outage_hold; do
 run_case "execution_$case_name" --inner pid --duration 5 --mode position --execution-case "$case_name"
done
