#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
work_dir="${RUNNER_TEMP:-/tmp}/estimator-hover-thrust-compliance"
install_root="${RUNNER_TEMP:-/tmp}/estimator-hover-thrust-install-root"

rm -rf "$work_dir" "$install_root"
mkdir -p "$work_dir/src/hover-thrust"
rsync -a --delete "$REPO_ROOT/" "$work_dir/src/hover-thrust/"

cd "$work_dir"
set +u
source /opt/ros/noetic/setup.bash
set -u
catkin_make run_tests_hover_thrust_estimator
catkin_test_results
DESTDIR="$install_root" catkin_make install \
  -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic \
  -DCMAKE_BUILD_TYPE=Release
"${SCRIPT_DIR}/check_core_libraries.sh" --install-root "$install_root"
set +u
source devel/setup.bash
set -u
test "$(rospack find hover_thrust_estimator)" = "$work_dir/src/hover-thrust/hover_thrust_estimator"
test "$(rospack find hover_thrust_estimator_msgs)" = "/opt/ros/noetic/share/hover_thrust_estimator_msgs"
roslaunch --files hover_thrust_estimator hover_thrust_estimator.launch >/tmp/xgc2-hover-thrust-estimator-files.txt
