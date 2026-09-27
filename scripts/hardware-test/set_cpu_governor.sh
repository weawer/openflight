#!/usr/bin/env bash
# Pin (or restore) the CPU frequency governor for latency-sensitive hardware
# tests. Frequency scaling under the default governor (ondemand/schedutil)
# adds latency and jitter while a core ramps up under load — exactly the kind
# of noise that would show up in trigger_delta_ms spread or self-trigger S!
# relay latency measurements. Run this before such a test, and `restore`
# after, rather than leaving the Pi pinned to `performance` permanently.
#
# Usage:
#   scripts/hardware-test/set_cpu_governor.sh            # pin to performance
#   scripts/hardware-test/set_cpu_governor.sh restore    # back to the saved governor

set -euo pipefail

GOVERNOR_GLOB=/sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
STATE_FILE="${TMPDIR:-/tmp}/openflight_cpu_governor.saved"

if ! compgen -G "$GOVERNOR_GLOB" >/dev/null; then
    echo "No scaling_governor files found; cpufreq scaling may be unavailable." >&2
    exit 1
fi

if [[ "${1:-}" == "restore" ]]; then
    if [[ ! -f "$STATE_FILE" ]]; then
        echo "No saved governor at $STATE_FILE; nothing to restore." >&2
        exit 1
    fi
    saved="$(cat "$STATE_FILE")"
    for f in $GOVERNOR_GLOB; do
        echo "$saved" | sudo tee "$f" >/dev/null
    done
    rm -f "$STATE_FILE"
    echo "Restored CPU governor to $saved."
    exit 0
fi

first_governor_file="$(compgen -G "$GOVERNOR_GLOB" | head -n1)"
cat "$first_governor_file" > "$STATE_FILE"
echo "Saved current governor ($(cat "$STATE_FILE")) to $STATE_FILE"

for f in $GOVERNOR_GLOB; do
    echo performance | sudo tee "$f" >/dev/null
done
echo "CPU governor pinned to performance. Run:"
echo "  scripts/hardware-test/set_cpu_governor.sh restore"
echo "afterward to return to the saved governor."
