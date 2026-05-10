#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="/c/Users/XY/Desktop/Drone"
COUNT="${1:-8}"

export HOME="$WORKSPACE/.tmp/msys_home"
export TMPDIR="$WORKSPACE/.tmp/msys_tmp"
export TMP="$TMPDIR"
export TEMP="$TMPDIR"
export PATH="$WORKSPACE/scripts/msys_bin:/usr/local/bin:/usr/bin:/bin:/c/Windows/System32:/c/Windows"
export PYTHONPATH="$WORKSPACE/.tmp/pydeps${PYTHONPATH:+:$PYTHONPATH}"
export SITL_RITW_TERMINAL="$WORKSPACE/scripts/msys_bin/ritw_exec"
export SITL_RITW_LOGDIR="$WORKSPACE/.tmp/sitl_ritw_logs"

mkdir -p "$HOME" "$TMPDIR" "$SITL_RITW_LOGDIR"

cd "$WORKSPACE/ardupilot"

python Tools/autotest/sim_vehicle.py \
  -v ArduCopter \
  -f quad \
  --no-mavproxy \
  --no-rebuild \
  --udp \
  --count "$COUNT" \
  --auto-sysid \
  -I0 \
  -L CMAC
