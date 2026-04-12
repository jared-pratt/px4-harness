#!/usr/bin/env bash
# start_sitl.sh — launch headless PX4 SITL instances (sihsim_quadx)
#
# Usage:
#   bash scripts/start_sitl.sh [LAT] [LON] [ALT_MSL] [NUM_DRONES]
#
# Defaults to your saved home location and 2 drones if no args given.
#   bash scripts/start_sitl.sh 40.769028 -111.846333 1300 3
#
# Each instance runs in its own gnome-terminal tab. If gnome-terminal is
# unavailable the script falls back to printing the commands to run manually.

set -euo pipefail

LAT="${1:-40.769028}"
LON="${2:--111.846333}"
ALT="${3:-1300}"
NUM="${4:-2}"

PX4_DIR="${PX4_DIR:-$HOME/ResearchFall2025/Simulators/PX4-Autopilot}"

if [ ! -d "$PX4_DIR" ]; then
    echo "ERROR: PX4-Autopilot not found at $PX4_DIR"
    echo "  Set PX4_DIR env var to the correct path."
    exit 1
fi

echo "==> Killing any stale PX4 processes and lock files..."
pkill -9 -f px4 2>/dev/null || true
sleep 1
rm -f /tmp/px4_lock-* /tmp/px4-sock-*
echo "    Done."

echo ""
echo "==> Starting $NUM PX4 instance(s) at LAT=$LAT LON=$LON ALT=${ALT}m MSL"

CMDS=()
for i in $(seq 0 $((NUM - 1))); do
    CMDS+=("cd $PX4_DIR && PX4_HOME_LAT=$LAT PX4_HOME_LON=$LON PX4_HOME_ALT=$ALT HEADLESS=1 make px4_sitl sihsim_quadx INSTANCE=$i")
done

if command -v gnome-terminal &>/dev/null; then
    TABS=()
    for i in $(seq 0 $((NUM - 1))); do
        TABS+=(--tab --title="PX4 drone$((i+1))" -- bash -c "${CMDS[$i]}; exec bash")
    done
    gnome-terminal "${TABS[@]}"
    echo "    Launched $NUM instances in gnome-terminal tabs."
else
    echo ""
    echo "gnome-terminal not found — run these in separate terminals:"
    echo ""
    for i in $(seq 0 $((NUM - 1))); do
        echo "  Terminal $((i+1)):  ${CMDS[$i]}"
    done
fi
