#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"

require_command() {
  local command_name="$1"
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "missing required command: ${command_name}" >&2
    exit 1
  fi
}

require_command file
require_command ldd

set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

dpkg -s ros-noetic-xgc2-estimator-hover-thrust >/dev/null
dpkg -s ros-noetic-xgc2-estimator-hover-thrust-msgs >/dev/null
dpkg -s libxgc2-math-dev >/dev/null
dpkg -s libxgc2-state-machine-dev >/dev/null
dpkg -s ros-noetic-xgc2-ros1-utils >/dev/null
dpkg -s ros-noetic-xgc2-state-machine-msgs >/dev/null
test -f /usr/include/xgc2_math/estimation/recursive_least_squares.hpp
test -f /usr/include/xgc2_math/filter/slew_rate_limiter.hpp
test -f /usr/include/xgc2_math/utils/sample_timing.hpp
test -f /usr/include/state_machine/state_machine.hpp
test -f "/opt/ros/${ROS_DISTRO}/include/state_machine_msgs/StateMachineTrace.h"
test "$(rospack find hover_thrust_estimator)" = "/opt/ros/${ROS_DISTRO}/share/hover_thrust_estimator"
test "$(rospack find hover_thrust_estimator_msgs)" = "/opt/ros/${ROS_DISTRO}/share/hover_thrust_estimator_msgs"
test "$(rospack find state_machine_msgs)" = "/opt/ros/${ROS_DISTRO}/share/state_machine_msgs"
test -f "/opt/ros/${ROS_DISTRO}/share/hover_thrust_estimator/config/hover_thrust_estimator.yaml"
test -f "/opt/ros/${ROS_DISTRO}/share/hover_thrust_estimator_msgs/msg/HoverThrustEstimate.msg"
test -f "/opt/ros/${ROS_DISTRO}/include/hover_thrust_estimator_msgs/HoverThrustEstimate.h"
test -f "/opt/ros/${ROS_DISTRO}/lib/pkgconfig/hover_thrust_estimator_msgs.pc"
test -f "/opt/ros/${ROS_DISTRO}/lib/pkgconfig/hover_thrust_estimator.pc"
test -f "/opt/ros/${ROS_DISTRO}/lib/python3/dist-packages/hover_thrust_estimator_msgs/msg/_HoverThrustEstimate.py"
test -f "/opt/ros/${ROS_DISTRO}/lib/libhover_thrust_estimator_math.so"
test -f "/opt/ros/${ROS_DISTRO}/lib/libhover_thrust_estimator_core.so"
rosmsg show hover_thrust_estimator_msgs/HoverThrustEstimate | grep -q '^float64 hover_thrust$'
python3 - <<'PY'
from hover_thrust_estimator_msgs.msg import HoverThrustEstimate

msg = HoverThrustEstimate()
msg.hover_thrust = 0.3
assert abs(msg.hover_thrust - 0.3) < 1e-12
PY
roslaunch --files hover_thrust_estimator hover_thrust_estimator.launch >/tmp/xgc2-hover-thrust-estimator-files.txt

while IFS= read -r file; do
  if ! file -b "${file}" | grep -q '^ELF'; then
    continue
  fi
  if ! ldd "${file}" | awk '/not found/ {missing=1} END {exit missing ? 1 : 0}'; then
    echo "missing shared library dependency in ${file}" >&2
    ldd "${file}" >&2 || true
    exit 1
  fi
done < <(find "/opt/ros/${ROS_DISTRO}/lib/hover_thrust_estimator" \
  "/opt/ros/${ROS_DISTRO}/lib/libhover_thrust_estimator_math.so" \
  "/opt/ros/${ROS_DISTRO}/lib/libhover_thrust_estimator_core.so" -type f 2>/dev/null | sort -u)

echo "Installed package check passed"
