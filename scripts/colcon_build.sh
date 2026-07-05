#!/bin/bash
# Canonical way to build this workspace. Always use this instead of calling
# `colcon build` directly — it bakes in two environment workarounds that are
# otherwise easy to lose track of and re-diagnose from scratch:
#
#  1. CMAKE_PREFIX_PATH includes cmake_fix/, a drop-in ament_cmake_target_dependencies
#     config that some vendored packages (e.g. irobot_create_nodes, turtlebot4_node)
#     need to find `ament_target_dependencies` on this ROS 2 Lyrical install.
#     Do NOT "fix" this by editing those vendored CMakeLists.txt files instead —
#     that was tried and just duplicates broken/incomplete copies of this macro.
#  2. GCC 15 on Lyrical treats warnings in third-party code (e.g. Navigation2) as
#     errors unless AMENT_CMAKE_CXX_WARNINGS_AS_ERRORS is off.
set -e
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export CMAKE_PREFIX_PATH="$REPO_ROOT/cmake_fix:$CMAKE_PREFIX_PATH"
colcon build --cmake-args -DAMENT_CMAKE_CXX_WARNINGS_AS_ERRORS=OFF "$@"
