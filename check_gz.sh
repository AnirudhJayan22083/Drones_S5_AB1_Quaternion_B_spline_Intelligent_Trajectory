#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

source .venv/bin/activate
echo "--- Testing Gazebo Python bindings ---"
python -c "import gz.sim8, gz.math7; print('gazebo ok (gz.sim8, gz.math7)')"
python -c "import gz.sim, gz.math; print('gazebo ok (gz.sim, gz.math)')"
python -c "from stage7_gazebo_sim.gazebo_backend import gz_sim, gz_math; print('backend import ok:', gz_sim, gz_math)"
