#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

dpkg -s ros-noetic-xgc2-estimator-hover-thrust >/dev/null
test "$(rospack find hover_thrust_estimator)" = "/opt/ros/${ROS_DISTRO}/share/hover_thrust_estimator"
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
done < <(find "/opt/ros/${ROS_DISTRO}/lib/hover_thrust_estimator" "/opt/ros/${ROS_DISTRO}/lib/libhover_thrust_estimator_core.so" -type f 2>/dev/null | sort -u)

echo "Installed package check passed"
