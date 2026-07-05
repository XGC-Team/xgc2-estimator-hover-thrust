#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-noetic}"
PREFIX="/opt/ros/${ROS_DISTRO}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root)
      PREFIX="$2/opt/ros/${ROS_DISTRO}"
      shift 2
      ;;
    --prefix)
      PREFIX="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

# ROS adapters are intentionally excluded from this boundary check:
# - include/hover_thrust_estimator/input and src/input
# - include/hover_thrust_estimator/output and src/output
# - hover_thrust_estimator_node and hover_thrust_estimator_main
# Those layers translate ROS topics/messages into the ROS-free runtime context.
core_paths=(
  "${REPO_ROOT}/hover_thrust_estimator/include/hover_thrust_estimator/common"
  "${REPO_ROOT}/hover_thrust_estimator/include/hover_thrust_estimator/state_machine"
  "${REPO_ROOT}/hover_thrust_estimator/include/hover_thrust_estimator/hover_thrust_estimator.h"
  "${REPO_ROOT}/hover_thrust_estimator/include/hover_thrust_estimator/hover_thrust_estimator_runtime.h"
  "${REPO_ROOT}/hover_thrust_estimator/include/hover_thrust_estimator/hover_thrust_output_model.h"
  "${REPO_ROOT}/hover_thrust_estimator/src/hover_thrust_estimator_runtime.cpp"
  "${REPO_ROOT}/hover_thrust_estimator/src/state_machine"
)

ros_source_pattern='(#include[[:space:]]*[<"][^>"]*(ros/|ros\.h|roscpp|rospy|[[:alnum:]_]+_msgs/|diagnostic_msgs/|tf2/|tf/)[^>"]*[>"])|(^|[^[:alnum:]_])(ros::|[[:alnum:]_]+_msgs::|diagnostic_msgs::|tf2::)'
if grep -RInE "${ros_source_pattern}" "${core_paths[@]}"; then
  echo "core hover thrust runtime, state, and context files must not depend on ROS APIs or ROS messages; ROS adapters may depend on ROS" >&2
  exit 1
fi

check_core_library() {
  local lib_path="$1"
  test -f "${lib_path}"
  local deps
  deps="$(LD_LIBRARY_PATH="${PREFIX}/lib:${LD_LIBRARY_PATH:-}" ldd "${lib_path}")"
  if grep -E 'not found' <<<"${deps}"; then
    echo "missing shared library dependency in ${lib_path}" >&2
    exit 1
  fi
  if grep -E '(libros|librostime|libroscpp|librosconsole|libgeometry_msgs|libsensor_msgs|libstd_msgs|libmavros_msgs|libdiagnostic_msgs|libtf2|libtf)' <<<"${deps}"; then
    echo "core library must not link against ROS: ${lib_path}" >&2
    exit 1
  fi
}

check_core_library "${PREFIX}/lib/libhover_thrust_estimator_math.so"

echo "Core library ROS-boundary check passed"
