#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../../.."
S=experiments/panda_contact_isaac/scripts
R=experiments/panda_contact_isaac/runs
for name in previous tight; do
 args=(--ik-seed previous)
 if [ "$name" = tight ]; then args+=(--ik-tolerance .000001); fi
 "$S/isaac-python" "$S/run.py" --inner pid --mode admittance --contact-gate --freeze-on-contact "${args[@]}" --out "$R/stability_ik_$name" > "$R/stability_ik_$name.log" 2>&1
 if [ "$name" = tight ]; then
  test -f "$R/stability_ik_$name/FAILED.json"
 else
  test -f "$R/stability_ik_$name/COMPLETE"
 fi
done
