#!/usr/bin/env bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

source .venv/bin/activate
echo "Starting Gazebo simulation with GUI viewer..."
python -m stage7_gazebo_sim.run_gazebo_demo --viewer
