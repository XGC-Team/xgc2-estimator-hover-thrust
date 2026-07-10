#!/usr/bin/env bash
set -euo pipefail

INSTALL_ROOT=""
OUTPUT_DIR=""
ROS_DISTRO="${ROS_DISTRO:-noetic}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ESTIMATOR_PACKAGE="ros-${ROS_DISTRO}-xgc2-estimator-hover-thrust"
ESTIMATOR_ROS_PACKAGE="hover_thrust_estimator"

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
Depends: ros-${ROS_DISTRO}-xgc2-estimator-hover-thrust-msgs (>= 1.2.0-3), libxgc2-math-dev (>= 0.5.6-5~focal), libxgc2-state-machine-dev (>= 0.1.3-4~focal), ros-${ROS_DISTRO}-xgc2-ros1-utils (>= 1.1.1-3), ros-${ROS_DISTRO}-xgc2-state-machine-msgs (>= 1.2.0-3), ros-${ROS_DISTRO}-roscpp, ros-${ROS_DISTRO}-sensor-msgs, ros-${ROS_DISTRO}-geometry-msgs, ros-${ROS_DISTRO}-mavros-msgs
Description: XGC2 hover thrust estimation package for PX4/MAVROS UAV controllers
EOF
printf 'xgc2-estimator-hover-thrust package\n' > "${estimator_pkg_root}/usr/share/doc/${ESTIMATOR_PACKAGE}/README"
find "${estimator_pkg_root}" -type d -exec chmod 0755 {} +
find "${estimator_pkg_root}" -type f -exec chmod 0644 {} +
chmod 0755 "${estimator_pkg_root}/DEBIAN"

fakeroot dpkg-deb --build "${estimator_pkg_root}" \
  "${OUTPUT_DIR}/${ESTIMATOR_PACKAGE}_${VERSION}_${ARCH}.deb" >/dev/null
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.deb' -print | sort
