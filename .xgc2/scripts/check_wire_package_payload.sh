#!/usr/bin/env bash
set -euo pipefail
[[ $# == 2 ]] || { echo 'usage: check_wire_package_payload.sh INSTALL_ROOT DEB_DIR' >&2; exit 2; }
install_root="$1" deb_dir="$2"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
work="$(mktemp -d)"
trap 'rm -rf -- "$work"' EXIT
shopt -s nullglob
debs=("$deb_dir"/libxgc2-hover-thrust-dev_*.deb)
[[ ${#debs[@]} == 1 ]] || { echo 'expected exactly one owning DTO Deb' >&2; exit 1; }
dpkg-deb --extract "${debs[0]}" "$work/payload"
WIRE_PATHS=(
  "/opt/ros/${ROS_DISTRO:-noetic}/include/hover_thrust_estimator/native/hover_thrust_wire.h"
  "/opt/ros/${ROS_DISTRO:-noetic}/share/cmake/HoverThrustNativeWire/HoverThrustNativeWireConfig.cmake"
  "/opt/ros/${ROS_DISTRO:-noetic}/share/cmake/HoverThrustNativeWire/HoverThrustNativeWireConfigVersion.cmake"
  "/opt/ros/${ROS_DISTRO:-noetic}/share/cmake/HoverThrustNativeWire/HoverThrustNativeWireTargets.cmake"
)
for path in "${WIRE_PATHS[@]}"; do
  cmp "$install_root$path" "$work/payload$path"
done
for missing in "${WIRE_PATHS[@]}"; do
  root="$work/missing"
  rm -rf -- "$root" "$work/out"
  mkdir -p "$root"
  cp -a "$install_root/." "$root/"
  rm -- "$root$missing"
  if "$script_dir/package_debs.sh" --install-root "$root" --output-dir "$work/out" >"$work/negative.log" 2>&1; then
    echo "packager accepted missing owning DTO export: $missing" >&2; exit 1
  fi
  grep -Fq "missing required installed owning DTO export: $missing" "$work/negative.log"
  [[ ! -d "$work/out" ]] || [[ -z "$(find "$work/out" -name '*.deb' -print -quit)" ]]
done
echo 'PASS: owning DTO Deb bytes and missing-header/export refusals'
