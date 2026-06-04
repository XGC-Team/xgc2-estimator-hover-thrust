#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-noetic}"

require_command() {
  local command_name="$1"
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "missing required command: ${command_name}" >&2
    exit 1
  fi
}

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "missing ROS setup: /opt/ros/${ROS_DISTRO}/setup.bash" >&2
  exit 1
fi

# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"

require_command clang-format
require_command clang-tidy
require_command catkin_make
require_command rsync

mapfile -d '' CXX_FILES < <(
  cd "${REPO_ROOT}"
  git ls-files -z -- '*.cpp' '*.cc' '*.cxx' '*.h' '*.hpp' '*.hh' '*.hxx'
)

if [[ "${#CXX_FILES[@]}" -eq 0 ]]; then
  echo "no C++ files found" >&2
  exit 1
fi

echo "Running clang-format..."
(
  cd "${REPO_ROOT}"
  clang-format --dry-run --Werror "${CXX_FILES[@]}"
)

WORK_DIR="${RUNNER_TEMP:-${TMPDIR:-/tmp}}/xgc2-hover-thrust-cpp-quality"
rm -rf "${WORK_DIR}"
mkdir -p "${WORK_DIR}/src/hover_thrust_estimator"
rsync -a --delete --exclude '.git' "${REPO_ROOT}/" "${WORK_DIR}/src/hover_thrust_estimator/"

echo "Generating compile_commands.json..."
(
  cd "${WORK_DIR}"
  catkin_make \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_BUILD_TYPE=Debug
)

echo "Running clang-tidy..."
TIDY_SOURCES=(
  "${WORK_DIR}/src/hover_thrust_estimator/src/hover_thrust_estimator_main.cpp"
  "${WORK_DIR}/src/hover_thrust_estimator/src/hover_thrust_estimator_node.cpp"
)

clang-tidy \
  -p "${WORK_DIR}/build" \
  -quiet \
  "${TIDY_SOURCES[@]}"

echo "C++ quality check passed"
