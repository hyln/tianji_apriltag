#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/../tianji_arm_teleop/config/global_config.sh"
export PYTHONPATH="$ROOT${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 -m tianji_apriltag.node "$@"
