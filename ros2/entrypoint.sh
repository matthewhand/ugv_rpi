#!/bin/bash
# Headless ROS 2 stack for Flask dual-path control (no RViz / no full ugv_ws required).
# Image: dudulrx0601/ugv_rpi_ros_humble:ugv_rpi_ros_humble
# Provides: rosbridge_websocket :9090 + ugv_driver_min (cmd_vel/joint_states → ESP32)
# Optional: roarm_driver_min (joint_command → USB RoArm) when ROARM_ENABLE_DRIVER=1
set -eo pipefail
# Note: do not use `set -u` before sourcing ROS setup (AMENT_* unbound vars)

log() { echo "[ugv_ros2 $(date -Is)] $*"; }

# shellcheck disable=SC1091
set +u
source /opt/ros/humble/setup.bash
set -u

export UGV_MODEL="${UGV_MODEL:-ugv_beast}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
SERIAL_DEV="${UGV_SERIAL_DEV:-/dev/ttyAMA0}"
ROARM_ENABLE_DRIVER="${ROARM_ENABLE_DRIVER:-0}"

log "UGV_MODEL=$UGV_MODEL serial=$SERIAL_DEV ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROARM_ENABLE_DRIVER=$ROARM_ENABLE_DRIVER"

if [[ ! -e "$SERIAL_DEV" ]]; then
  log "WARNING: $SERIAL_DEV missing — ugv_driver will retry"
fi

if ! ros2 pkg prefix rosbridge_server >/dev/null 2>&1; then
  log "ERROR: rosbridge_server not installed"
  exit 1
fi

DRIVER="${UGV_DRIVER_SCRIPT:-/opt/ugv_ros2/ugv_driver_min.py}"
if [[ ! -f "$DRIVER" ]]; then
  log "ERROR: driver script not found at $DRIVER"
  exit 1
fi

ROARM_DRIVER="${ROARM_DRIVER_SCRIPT:-/opt/ugv_ros2/roarm_driver_min.py}"

cleanup() {
  log "shutting down children"
  kill $(jobs -p) 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

log "starting rosbridge_websocket"
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
RB_PID=$!

sleep 2

log "starting ugv_driver_min ($DRIVER)"
python3 "$DRIVER" &
DRV_PID=$!

ROARM_PID=""
if [[ "$ROARM_ENABLE_DRIVER" == "1" || "$ROARM_ENABLE_DRIVER" == "true" || "$ROARM_ENABLE_DRIVER" == "yes" ]]; then
  if [[ -f "$ROARM_DRIVER" ]]; then
    ROARM_DEV="${ROARM_SERIAL:-/dev/ttyUSB0}"
    if [[ -e "$ROARM_DEV" ]] || [[ -e /dev/ttyUSB0 ]] || ls /dev/serial/by-id/usb-Silicon_Labs_CP2102N_* >/dev/null 2>&1; then
      log "starting roarm_driver_min ($ROARM_DRIVER) ROARM_SERIAL=${ROARM_SERIAL:-auto}"
      # Fail soft: if USB busy/missing, node retries; do not kill chassis stack
      python3 "$ROARM_DRIVER" &
      ROARM_PID=$!
    else
      log "WARNING: ROARM_ENABLE_DRIVER set but no CP2102/ttyUSB found — skipping RoArm driver"
    fi
  else
    log "WARNING: RoArm driver script missing at $ROARM_DRIVER — skipping"
  fi
else
  log "RoArm in-container driver disabled (ROARM_ENABLE_DRIVER=0); Flask hybrid USB or host start_roarm_driver.sh"
fi

log "stack up: rosbridge_pid=$RB_PID driver_pid=$DRV_PID roarm_pid=${ROARM_PID:-none}"
# Keep container alive while rosbridge is up.
# Chassis driver may be stopped by Flask (Direct UART reclaim) and later
# restarted via docker exec — do not tear down rosbridge if the driver exits.
# RoArm driver is optional: its exit does not tear down the stack.
# Do not auto-restart ugv_driver_min here: that would steal ttyAMA0 from Flask.
while kill -0 "$RB_PID" 2>/dev/null; do
  # Soft-restart RoArm driver if it died while still enabled
  if [[ -n "${ROARM_PID}" ]]; then
    if ! kill -0 "$ROARM_PID" 2>/dev/null; then
      log "roarm_driver exited — restarting in 3s (fail soft)"
      sleep 3
      if [[ "$ROARM_ENABLE_DRIVER" == "1" || "$ROARM_ENABLE_DRIVER" == "true" || "$ROARM_ENABLE_DRIVER" == "yes" ]]; then
        python3 "$ROARM_DRIVER" &
        ROARM_PID=$!
        log "roarm_driver restarted pid=$ROARM_PID"
      fi
    fi
  fi
  sleep 2
done
log "rosbridge exited (rb=down drv=$(kill -0 $DRV_PID 2>/dev/null && echo up || echo down))"
exit 1
