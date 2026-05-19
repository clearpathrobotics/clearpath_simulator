#!/usr/bin/env bash
# Generate the flattened A300 URDF and stage meshes so the isaac-sim container
# can find them under /workspace.
#
# Run from the host with ROS 2 Jazzy + clearpath_common sourced:
#   source /opt/ros/jazzy/setup.bash
#   source ~/your_ws/install/setup.bash
#   ./scripts/generate_urdf.sh

set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
URDF_OUT="${PKG_DIR}/urdf/a300.urdf"
MESH_DIR="${PKG_DIR}/meshes/clearpath_platform_description/meshes/a300"

PLATFORM_SHARE="$(ros2 pkg prefix clearpath_platform_description)/share/clearpath_platform_description"

echo "[1/3] xacro -> ${URDF_OUT}"
empty_yaml="$(mktemp)"
trap 'rm -f "${empty_yaml}"' EXIT
xacro "${PKG_DIR}/urdf/a300_standalone.urdf.xacro" \
    is_sim:=true \
    use_platform_controllers:=false \
    gazebo_controllers:="${empty_yaml}" \
    > "${URDF_OUT}"

echo "[2/3] rewriting mesh paths"
# package://clearpath_platform_description/... -> file:///workspace/meshes/clearpath_platform_description/...
sed -i 's|package://clearpath_platform_description/|file:///workspace/meshes/clearpath_platform_description/|g' "${URDF_OUT}"

echo "[3/3] staging meshes into ${MESH_DIR}"
mkdir -p "${MESH_DIR}"
# Dereference symlinks (colcon install often symlinks share/) so the meshes
# are real files inside the bind mount and visible from the container.
cp -rL "${PLATFORM_SHARE}/meshes/a300/." "${MESH_DIR}/"

echo "done. next: docker compose -f docker/compose.yaml exec isaac-sim /isaac-sim/python.sh /workspace/scripts/import_a300.py"
