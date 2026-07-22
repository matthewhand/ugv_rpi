#!/bin/bash
# Dev server for Waveshare UGV web UI with hot reload on :5000.
# Usage: ./run_dev.sh
#   UGV_MOTOR_BYPASS=0 ./run_dev.sh   # allow direct motor cmds (default: bypass for ROS 2)
#   UGV_PORT=5000 ./run_dev.sh

set -euo pipefail
cd "$(dirname "$0")"

export UGV_HOT_RELOAD="${UGV_HOT_RELOAD:-1}"
# Process reloader off by default: serial/camera re-init on every *.py save is painful
# next to ROS 2. Set UGV_RELOADER=1 when actively editing app.py.
export UGV_RELOADER="${UGV_RELOADER:-0}"
export UGV_MOTOR_BYPASS="${UGV_MOTOR_BYPASS:-1}"
export UGV_PORT="${UGV_PORT:-5000}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"

if [[ -x ./ugv-env/bin/python ]]; then
  PY=./ugv-env/bin/python
else
  PY=python3
fi

# Pull OPENAI_* etc. from .env if present (python-dotenv also loads inside app.py)
if [[ -f .env ]]; then
  set -a
  # shellcheck disable=SC1091
  source .env
  set +a
fi

echo "Starting app.py HOT_RELOAD=$UGV_HOT_RELOAD RELOADER=$UGV_RELOADER MOTOR_BYPASS=$UGV_MOTOR_BYPASS port=$UGV_PORT"
echo "LLM: ${OPENAI_MODEL:-?} @ ${OPENAI_BASE_URL:-?}"
exec "$PY" app.py
