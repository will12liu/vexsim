#!/bin/sh
# Copy the auton routine from the robot code (will12liu/titaniumvex, branch main) into the sim.
# The sim loads routine.cpp at startup. Run from anywhere: sh tools/sync-robot-code.sh [path/to/titaniumvex]
set -e
SIM="$(cd "$(dirname "$0")/.." && pwd)"
ROBOT="${1:-$SIM/../titaniumvex}"
git -C "$ROBOT" fetch -q origin
git -C "$ROBOT" show origin/main:src/routine.cpp > "$SIM/routine.cpp"
echo "routine.cpp <- titaniumvex origin/main ($(git -C "$ROBOT" log -1 --format='%h %cd' --date=short origin/main -- src/routine.cpp))"
