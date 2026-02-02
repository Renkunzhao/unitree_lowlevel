#!/usr/bin/env bash
set -eo pipefail

# Usage: ros2_task.sh <network_interface> <command...>
# Example:
#   ros2_task.sh lo ros2 run plotjuggler plotjuggler
#   ros2_task.sh eth0 ros2 launch pkg file.launch.py arg:=val

if [[ $# -lt 2 ]]; then
  echo "Usage: ros2_task.sh <network_interface> <command...>" >&2
  exit 2
fi

NET_IF="$1"
shift

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETUP_SH="$SCRIPT_DIR/setup.sh"

if [[ -f "$SETUP_SH" ]]; then
  # shellcheck disable=SC1090
  source "$SETUP_SH" "$NET_IF" "$ROS_DISTRO"
else
  echo "[ros2_task] Warning: $SETUP_SH not found; continuing without sourcing" >&2
fi

exec "$@"
