#!/usr/bin/env bash
# export_lite6_urdf_for_unity.sh
#
# Produces a self-contained Lite 6 URDF folder that Unity's URDF-Importer can
# consume directly. The stock xarm_description URDFs use `package://` mesh
# paths which Unity does not resolve; this script:
#   1. Runs xacro to expand the Lite 6 xacro into a flat URDF
#   2. Copies all referenced .stl / .dae files alongside
#   3. Rewrites mesh filenames in the URDF to relative paths Unity understands
#
# Usage:
#   ./export_lite6_urdf_for_unity.sh [OUTPUT_DIR]
# Default output: ~/lite6_urdf_for_unity/
#
# Requirements:
#   - ros2 jazzy sourced (so `ros2 pkg prefix xarm_description` works)
#   - `xacro` available (installed with ros-jazzy-xacro or via the xarm_ros2 deps)

set -euo pipefail

OUTPUT_DIR="${1:-$HOME/lite6_urdf_for_unity}"
mkdir -p "${OUTPUT_DIR}/meshes"

if ! command -v ros2 &>/dev/null; then
    echo "ERROR: ros2 not on PATH — did you source /opt/ros/jazzy/setup.bash?" >&2
    exit 1
fi

XARM_DESC_DIR="$(ros2 pkg prefix xarm_description)/share/xarm_description"
if [[ ! -d "${XARM_DESC_DIR}" ]]; then
    echo "ERROR: xarm_description not found at ${XARM_DESC_DIR}" >&2
    echo "  Did you source your ros2_ws? Run: source ~/ros2_ws/install/setup.bash" >&2
    exit 1
fi

XACRO_FILE="${XARM_DESC_DIR}/urdf/lite6/lite6.urdf.xacro"
if [[ ! -f "${XACRO_FILE}" ]]; then
    echo "ERROR: xacro not found at ${XACRO_FILE}" >&2
    exit 1
fi

URDF_OUT="${OUTPUT_DIR}/lite6.urdf"

echo "==> Expanding xacro to URDF..."
# Some xacros expect positional args like dof, robot_type. Try expanding
# standalone first; if it complains about unset args, pass the Lite 6 ones.
if ! xacro "${XACRO_FILE}" prefix:='' \
        > "${URDF_OUT}" 2>/tmp/xacro_err; then
    echo "xacro failed with no args, trying with explicit prefix/add_gripper..."
    xacro "${XACRO_FILE}" prefix:='' add_gripper:=false ros2_control_plugin:='gazebo' \
        > "${URDF_OUT}" 2>/tmp/xacro_err || {
            echo "xacro failed. stderr:" >&2
            cat /tmp/xacro_err >&2
            exit 1
        }
fi
echo "    wrote ${URDF_OUT}"

echo "==> Locating referenced mesh files..."
# Extract all `package://...` mesh references, resolve them to real paths
# under /share, copy into our output meshes/ dir.
MESH_REFS=$(grep -oE 'package://[^"]+' "${URDF_OUT}" | sort -u)

COUNT=0
for ref in ${MESH_REFS}; do
    # ref looks like: package://xarm_description/meshes/lite6/visual/link_base.stl
    # strip 'package://<pkg>/' -> 'meshes/lite6/visual/link_base.stl'
    pkg=$(echo "${ref}" | sed -E 's|package://([^/]+)/.*|\1|')
    relpath=$(echo "${ref}" | sed -E 's|package://[^/]+/||')

    # Find the real file path under that package's share dir
    src_share="$(ros2 pkg prefix "${pkg}" 2>/dev/null || true)"
    if [[ -z "${src_share}" ]]; then
        echo "    WARN: package '${pkg}' not found, skipping ${ref}" >&2
        continue
    fi
    src="${src_share}/share/${pkg}/${relpath}"
    if [[ ! -f "${src}" ]]; then
        echo "    WARN: file not found: ${src}" >&2
        continue
    fi

    # Flatten into meshes/ using basename, preserving extension.
    # If two meshes share the same basename, disambiguate with parent dir.
    base="$(basename "${src}")"
    dest="${OUTPUT_DIR}/meshes/${base}"
    if [[ -f "${dest}" ]] && ! cmp -s "${src}" "${dest}"; then
        parent="$(basename "$(dirname "${src}")")"
        base="${parent}_${base}"
        dest="${OUTPUT_DIR}/meshes/${base}"
    fi
    cp -u "${src}" "${dest}"
    COUNT=$((COUNT + 1))

    # Rewrite the reference in the URDF to be relative
    #   package://xarm_description/meshes/lite6/visual/link_base.stl
    # becomes
    #   meshes/link_base.stl   (or meshes/visual_link_base.stl if disambiguated)
    esc_ref=$(printf '%s' "${ref}" | sed 's/[.[\*^$(){}?+|/]/\\&/g')
    sed -i "s|${esc_ref}|meshes/${base}|g" "${URDF_OUT}"
done
echo "    copied ${COUNT} mesh file(s)"

echo "==> Verifying no package:// references remain..."
if grep -q 'package://' "${URDF_OUT}"; then
    echo "WARN: some package:// references remain in URDF:" >&2
    grep -n 'package://' "${URDF_OUT}" | head >&2
else
    echo "    all mesh paths rewritten"
fi

echo "==> Summary"
echo "    URDF:    ${URDF_OUT}"
echo "    Meshes:  ${OUTPUT_DIR}/meshes ($(ls "${OUTPUT_DIR}/meshes" | wc -l) files)"
echo ""
echo "In Unity: Assets/URDF/lite6/  <-- copy this whole folder there"
echo "Then right-click lite6.urdf and choose: Import Robot from URDF"
