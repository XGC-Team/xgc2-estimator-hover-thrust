#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

dpkg -s ros-noetic-xgc2-estimator-hover-thrust >/dev/null
dpkg -s libxgc2-observer-dev >/dev/null
dpkg -s libxgc2-state-machine-dev >/dev/null
test -f /usr/include/xgc2_observer/recursive_least_squares.hpp
test -f /usr/include/state_machine/state_machine.hpp
test "$(rospack find hover_thrust_estimator)" = "/opt/ros/${ROS_DISTRO}/share/hover_thrust_estimator"
test -f "/opt/ros/${ROS_DISTRO}/share/hover_thrust_estimator/msg/HoverThrustEstimate.msg"
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
