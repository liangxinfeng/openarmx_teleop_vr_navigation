#!/usr/bin/env bash
# Keep the installed task_actions.txt pointing at the editable source copy.
#
# Run this AFTER every `colcon build` (colcon replaces the installed config with
# a regular copy, which is what caused the 2026-10-30 src/install divergence:
# src had the 7-point chain while install still had the 12-point one).
#
# Usage: link_task_catalog.sh [source] [installed]
set -Eeuo pipefail

SRC="${1:-/home/ubuntu/ros2_ws/src/openarmx_teleop_vr_navigation_306/config/task_actions.txt}"
INST="${2:-/home/ubuntu/ros2_ws/install/openarmx_teleop_vr_navigation_306/share/openarmx_teleop_vr_navigation_306/config/task_actions.txt}"

if [ ! -f "$SRC" ]; then
  printf 'missing source task catalog: %s\n' "$SRC" >&2
  exit 2
fi

mkdir -p "$(dirname "$INST")"

if [ -L "$INST" ] && [ "$(readlink -f "$INST")" = "$(readlink -f "$SRC")" ]; then
  printf 'already linked: %s -> %s\n' "$INST" "$SRC"
  exit 0
fi

if [ -e "$INST" ] && [ ! -L "$INST" ]; then
  backup="${INST}.bak.$(date +%Y%m%d_%H%M%S)"
  mv "$INST" "$backup"
  printf 'backup: %s\n' "$backup"
fi

ln -sfn "$SRC" "$INST"

if [ "$(readlink -f "$INST")" != "$(readlink -f "$SRC")" ]; then
  printf 'link verification failed: %s\n' "$INST" >&2
  exit 3
fi

ls -l "$INST"
printf 'OK: installed task catalog now follows the editable source\n'
