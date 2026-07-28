#!/bin/bash
# Start RoArm USB → ROS bridge without stealing base UART (ttyAMA0).
#
# Modes (first that works):
#   1) Host rosbridge bridge (no rclpy) — preferred on this Pi
#   2) Host rclpy node if available
#   3) Hint: enable ROARM_ENABLE_DRIVER=1 in ugv_ros2 compose
#
# Usage:
#   ~/ugv_rpi/ros2/start_roarm_driver.sh
#   ROARM_SERIAL=/dev/ttyUSB0 ~/ugv_rpi/ros2/start_roarm_driver.sh
#
# Stop: kill the printed PID or pkill -f roarm_bridge_host|roarm_driver_min
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ROS2_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"

export ROSBRIDGE_URL="${ROSBRIDGE_URL:-ws://127.0.0.1:9090}"
export ROARM_BAUD="${ROARM_BAUD:-115200}"
# Leave ROARM_SERIAL empty for by-id auto-detect in roarm_ctrl / driver

PY="${ROOT}/ugv-env/bin/python"
if [[ ! -x "$PY" ]]; then
  PY="$(command -v python3)"
fi

echo "[start_roarm_driver] ROOT=$ROOT python=$PY"
echo "[start_roarm_driver] ROSBRIDGE_URL=$ROSBRIDGE_URL"

# Prefer host bridge (websocket-client + pyserial; no rclpy)
if "$PY" -c "import websocket, serial" 2>/dev/null; then
  echo "[start_roarm_driver] starting host rosbridge bridge (roarm_bridge_host.py)"
  exec "$PY" "$ROS2_DIR/roarm_bridge_host.py"
fi

# Optional: system/container rclpy
if python3 -c "import rclpy, serial" 2>/dev/null; then
  echo "[start_roarm_driver] starting rclpy roarm_driver_min.py"
  exec python3 "$ROS2_DIR/roarm_driver_min.py"
fi

echo "[start_roarm_driver] ERROR: need websocket-client+pyserial (ugv-env) or rclpy" >&2
echo "  Install host bridge deps, or run inside ugv_ros2 with ROARM_ENABLE_DRIVER=1" >&2
exit 1
