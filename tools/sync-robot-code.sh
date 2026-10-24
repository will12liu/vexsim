#!/bin/sh
# Copy the auton routine from the robot code (will12liu/4787T-Titanium-Vex-Override, branch main) into
# the sim. The sim loads routine.cpp at startup. Run from anywhere:
#   sh tools/sync-robot-code.sh [path/to/4787T-Titanium-Vex-Override]
# Without a path it uses a checkout next to this one, or clones the (public) repo into a temp folder.
set -e
SIM="$(cd "$(dirname "$0")/.." && pwd)"
ROBOT="${1:-$SIM/../4787T-Titanium-Vex-Override}"
if [ ! -d "$ROBOT/.git" ]; then
	ROBOT="$(mktemp -d)/robot"
	git clone -q --depth 1 https://github.com/will12liu/4787T-Titanium-Vex-Override "$ROBOT"
fi
git -C "$ROBOT" fetch -q origin main
git -C "$ROBOT" show origin/main:src/routine.cpp > "$SIM/routine.cpp"
echo "routine.cpp <- 4787T-Titanium-Vex-Override origin/main ($(git -C "$ROBOT" log -1 --format='%h %cd' --date=short origin/main -- src/routine.cpp))"
