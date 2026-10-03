#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
INSTALL_ROOT="" OUTPUT_DIR="" WITH_ROS=true
ROS_DISTRO="${ROS_DISTRO:-noetic}"
PREFIX="/opt/ros/${ROS_DISTRO}"
VERSION="${PACKAGE_VERSION:-$(awk '$1 == "focal:" {print $2; exit}' "${REPO_ROOT}/.xgc2/product.yml")}"
[[ -n "$VERSION" ]] || { echo 'missing authoritative focal package version' >&2; exit 2; }
dpkg --validate-version "$VERSION"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root) INSTALL_ROOT="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    --prefix) PREFIX="$2"; shift 2 ;;
    --without-ros) WITH_ROS=false; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ -n "$INSTALL_ROOT" && -n "$OUTPUT_DIR" ]] || { echo '--install-root and --output-dir required' >&2; exit 2; }
ARCH="$(dpkg --print-architecture)"
WIRE_PATHS=(
  "${PREFIX}/include/hover_thrust_estimator/native/hover_thrust_wire.h"
  "${PREFIX}/share/cmake/HoverThrustNativeWire/HoverThrustNativeWireConfig.cmake"
  "${PREFIX}/share/cmake/HoverThrustNativeWire/HoverThrustNativeWireConfigVersion.cmake"
  "${PREFIX}/share/cmake/HoverThrustNativeWire/HoverThrustNativeWireTargets.cmake"
)
for path in "${WIRE_PATHS[@]}"; do
  [[ -f "${INSTALL_ROOT}${path}" ]] || {
    echo "missing required installed owning DTO export: ${path}" >&2
    exit 1
  }
done
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
mkdir -p "$OUTPUT_DIR"
core=libxgc2-hover-thrust-core0
dev=libxgc2-hover-thrust-dev
native=xgc2-hover-thrust-native
ros="ros-${ROS_DISTRO}-xgc2-estimator-hover-thrust"
old_boundary="$VERSION"
copy_required() {
  local source="$INSTALL_ROOT$1" target="$WORK/$2$1"
  [[ -e "$source" || -L "$source" ]] || { echo "missing installed owner artifact: $source" >&2; exit 1; }
  mkdir -p "$(dirname "$target")"
  cp -a "$source" "$target"
}
control() {
  local name="$1" depends="$2" description="$3" section="$4"
  mkdir -p "$WORK/$name/DEBIAN"
  cat > "$WORK/$name/DEBIAN/control" <<CONTROL
Package: $name
Version: $VERSION
Section: $section
Priority: optional
Architecture: $ARCH
Maintainer: XGC2 <apt@example.com>
Depends: $depends
Description: $description
CONTROL
}
# One actual shared domain runtime; neither facade embeds or packages a copy.
copy_required "$PREFIX/lib/libhover_thrust_estimator_math.so.0" "$core"
copy_required "$PREFIX/lib/libhover_thrust_estimator_math.so.1.1.24" "$core"
control "$core" 'libxgc2-state-machine-dev (>= 0.1.3-4~focal), libc6, libstdc++6, libgcc-s1' 'XGC2 ROS-free hover-thrust shared runtime' libs
printf 'Breaks: %s (<< %s)\nReplaces: %s (<< %s)\n' "$ros" "$old_boundary" "$ros" "$old_boundary" >> "$WORK/$core/DEBIAN/control"
printf 'libhover_thrust_estimator_math 0 %s (= %s)\n' "$core" "$VERSION" > "$WORK/$core/DEBIAN/shlibs"
copy_required "$PREFIX/lib/libhover_thrust_estimator_math.so" "$dev"
copy_required "$PREFIX/lib/cmake/HoverThrustEstimator" "$dev"
for path in "${WIRE_PATHS[@]}"; do copy_required "$path" "$dev"; done
for component in common state_machine; do copy_required "$PREFIX/include/hover_thrust_estimator/$component" "$dev"; done
for header in hover_thrust_estimator.h hover_thrust_estimator_runtime.h hover_thrust_output_model.h; do copy_required "$PREFIX/include/hover_thrust_estimator/$header" "$dev"; done
control "$dev" "$core (= $VERSION), libxgc2-math-dev (>= 0.5.6-6~focal), libxgc2-state-machine-dev (>= 0.1.3-4~focal)" 'XGC2 hover-thrust domain C++ headers and CMake target' libdevel
printf 'Breaks: %s (<< %s)\nReplaces: %s (<< %s)\n' "$ros" "$old_boundary" "$ros" "$old_boundary" >> "$WORK/$dev/DEBIAN/control"
copy_required "$PREFIX/lib/xgc-runtime/plugins/libest_hover_thrust.so" "$native"
control "$native" "$core (= $VERSION), libc6, libstdc++6, libgcc-s1" 'XGC2 hover-thrust native transport facade' libs
# The header-only runtime SDK is BUILD-only; there is no runtime Depends on it.
if "$WITH_ROS"; then
  copy_required "$PREFIX/share/hover_thrust_estimator" "$ros"
  copy_required "$PREFIX/lib/hover_thrust_estimator" "$ros"
  copy_required "$PREFIX/lib/pkgconfig/hover_thrust_estimator.pc" "$ros"
  copy_required "$PREFIX/lib/libhover_thrust_estimator_core.so" "$ros"
  for component in input output; do copy_required "$PREFIX/include/hover_thrust_estimator/$component" "$ros"; done
  copy_required "$PREFIX/include/hover_thrust_estimator/hover_thrust_estimator_node.h" "$ros"
  control "$ros" "$core (= $VERSION), $dev (= $VERSION), ros-${ROS_DISTRO}-xgc2-estimator-hover-thrust-msgs (>= 1.2.0-3), ros-${ROS_DISTRO}-xgc2-ros1-utils (>= 1.1.1-3), ros-${ROS_DISTRO}-xgc2-state-machine-msgs (>= 1.2.0-3), ros-${ROS_DISTRO}-roscpp, ros-${ROS_DISTRO}-sensor-msgs, ros-${ROS_DISTRO}-geometry-msgs, ros-${ROS_DISTRO}-mavros-msgs" 'XGC2 hover-thrust ROS1 transport facade' misc
fi
for name in "$core" "$dev" "$native" $("$WITH_ROS" && printf '%s' "$ros"); do
  artifact="$OUTPUT_DIR/${name}_${VERSION}_${ARCH}.deb"
  [[ ! -e "$artifact" ]] || { echo "refusing existing artifact: $artifact" >&2; exit 2; }
  dpkg-deb --root-owner-group --build "$WORK/$name" "$artifact"
done
