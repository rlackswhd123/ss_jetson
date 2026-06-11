#!/usr/bin/env bash
set -euo pipefail

mkdir -p "$SS_DEPTH_STATE_DIR"
echo "$$ $SS_DEPTH_TITLE" >> "$SS_DEPTH_STATE_DIR/pids"

exec bash -lc "$SS_DEPTH_SETUP_COMMAND && exec $SS_DEPTH_COMMAND"
