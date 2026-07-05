#!/usr/bin/env bash
set -euo pipefail

INSTALL_ROOT=""
OUTPUT_DIR=""
ROS_DISTRO="${ROS_DISTRO:-noetic}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ESTIMATOR_PACKAGE="ros-noetic-xgc2-estimator-hover-thrust"
MSGS_PACKAGE="ros-noetic-xgc2-estimator-hover-thrust-msgs"
ESTIMATOR_ROS_PACKAGE="hover_thrust_estimator"
MSGS_ROS_PACKAGE="hover_thrust_estimator_msgs"

product_version() {
  awk -F': *' '/^version:[[:space:]]*/ {print $2; exit}' "${REPO_ROOT}/.xgc2/product.yml"
}

VERSION="${PACKAGE_VERSION:-$(product_version)}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root)
      INSTALL_ROOT="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ -z "${INSTALL_ROOT}" || -z "${OUTPUT_DIR}" ]]; then
  echo "--install-root and --output-dir are required" >&2
  exit 1
fi

if [[ -z "${VERSION}" ]]; then
  echo "package version is missing" >&2
  exit 1
fi

ARCH="$(dpkg --print-architecture)"
PREFIX="/opt/ros/${ROS_DISTRO}"
PREFIX_ROOT="${INSTALL_ROOT}${PREFIX}"
BUILD_DIR="$(mktemp -d)"

cleanup() {
  rm -rf "${BUILD_DIR}"
}
trap cleanup EXIT

mkdir -p "${OUTPUT_DIR}"
rm -f "${OUTPUT_DIR}"/*.deb

copy_path() {
  local src="$1"
  local dst_root="$2"
  if [[ -e "${src}" ]]; then
    mkdir -p "${dst_root}$(dirname "${src#${INSTALL_ROOT}}")"
    cp -a "${src}" "${dst_root}${src#${INSTALL_ROOT}}"
  fi
}

build_deb() {
  local pkg_root="$1"
  local package_name="$2"
  fakeroot dpkg-deb --build "${pkg_root}" \
    "${OUTPUT_DIR}/${package_name}_${VERSION}_${ARCH}.deb" >/dev/null
}

msgs_pkg_root="${BUILD_DIR}/${MSGS_PACKAGE}"
mkdir -p "${msgs_pkg_root}"

copy_path "${PREFIX_ROOT}/share/${MSGS_ROS_PACKAGE}" "${msgs_pkg_root}"
copy_path "${PREFIX_ROOT}/include/${MSGS_ROS_PACKAGE}" "${msgs_pkg_root}"
copy_path "${PREFIX_ROOT}/lib/pkgconfig/${MSGS_ROS_PACKAGE}.pc" "${msgs_pkg_root}"
copy_path "${PREFIX_ROOT}/lib/python3/dist-packages/${MSGS_ROS_PACKAGE}" "${msgs_pkg_root}"
copy_path "${PREFIX_ROOT}/share/gennodejs/ros/${MSGS_ROS_PACKAGE}" "${msgs_pkg_root}"
copy_path "${PREFIX_ROOT}/share/common-lisp/ros/${MSGS_ROS_PACKAGE}" "${msgs_pkg_root}"
copy_path "${PREFIX_ROOT}/share/roseus/ros/${MSGS_ROS_PACKAGE}" "${msgs_pkg_root}"

mkdir -p "${msgs_pkg_root}/DEBIAN" "${msgs_pkg_root}/usr/share/doc/${MSGS_PACKAGE}"
cat > "${msgs_pkg_root}/DEBIAN/control" <<EOF
Package: ${MSGS_PACKAGE}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <apt@example.com>
Depends: ros-noetic-message-runtime, ros-noetic-std-msgs
Description: XGC2 hover thrust estimation message interfaces
EOF
printf 'xgc2-estimator-hover-thrust message package\n' > "${msgs_pkg_root}/usr/share/doc/${MSGS_PACKAGE}/README"
chmod 0755 "${msgs_pkg_root}/DEBIAN"
build_deb "${msgs_pkg_root}" "${MSGS_PACKAGE}"

estimator_pkg_root="${BUILD_DIR}/${ESTIMATOR_PACKAGE}"
mkdir -p "${estimator_pkg_root}"

copy_path "${PREFIX_ROOT}/share/${ESTIMATOR_ROS_PACKAGE}" "${estimator_pkg_root}"
copy_path "${PREFIX_ROOT}/lib/${ESTIMATOR_ROS_PACKAGE}" "${estimator_pkg_root}"
copy_path "${PREFIX_ROOT}/lib/pkgconfig/${ESTIMATOR_ROS_PACKAGE}.pc" "${estimator_pkg_root}"
copy_path "${PREFIX_ROOT}/include/${ESTIMATOR_ROS_PACKAGE}" "${estimator_pkg_root}"
copy_path "${PREFIX_ROOT}/lib/libhover_thrust_estimator_math.so" "${estimator_pkg_root}"
copy_path "${PREFIX_ROOT}/lib/libhover_thrust_estimator_core.so" "${estimator_pkg_root}"

mkdir -p "${estimator_pkg_root}/DEBIAN" "${estimator_pkg_root}/usr/share/doc/${ESTIMATOR_PACKAGE}"
cat > "${estimator_pkg_root}/DEBIAN/control" <<EOF
Package: ${ESTIMATOR_PACKAGE}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <apt@example.com>
Depends: ${MSGS_PACKAGE} (= ${VERSION}), libxgc2-math-dev (>= 0.5.6-1), libxgc2-state-machine-dev (>= 0.1.3-1~focal), ros-noetic-xgc2-ros1-utils, ros-noetic-xgc2-state-machine-msgs, ros-noetic-roscpp, ros-noetic-sensor-msgs, ros-noetic-geometry-msgs, ros-noetic-mavros-msgs
Description: XGC2 hover thrust estimation package for PX4/MAVROS UAV controllers
EOF
printf 'xgc2-estimator-hover-thrust package\n' > "${estimator_pkg_root}/usr/share/doc/${ESTIMATOR_PACKAGE}/README"
chmod 0755 "${estimator_pkg_root}/DEBIAN"
build_deb "${estimator_pkg_root}" "${ESTIMATOR_PACKAGE}"

find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.deb' -print | sort
