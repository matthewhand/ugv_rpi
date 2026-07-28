#!/bin/bash
# Start/upgrade the ugv_ros2 container. Releases UART expectations to the stack.
set -euo pipefail
cd "$(dirname "$0")"
chmod +x entrypoint.sh

IMAGE="dudulrx0601/ugv_rpi_ros_humble:ugv_rpi_ros_humble"
NAME="${UGV_ROS_CONTAINER:-ugv_ros2}"

echo "Pulling $IMAGE ..."
docker pull "$IMAGE"

# Remove stale container with wrong name/config if present
if docker inspect "$NAME" >/dev/null 2>&1; then
  echo "Recreating existing container $NAME"
  docker rm -f "$NAME" >/dev/null
fi

docker compose up -d
echo "Container status:"
docker ps --filter "name=^/${NAME}$" --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}\t{{.Ports}}'
echo "Logs (tail):"
docker logs --tail 40 "$NAME" || true
