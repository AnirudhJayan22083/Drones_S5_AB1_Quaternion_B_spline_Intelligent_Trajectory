"""
Single entry point that runs Stages 1-4 end to end and makes sure each
stage's demo plot exists in outputs/ -- generating only the ones that are
missing (or all of them with --force), instead of the usual workflow of
running four separate ``python -m stageN_...`` commands by hand.

Lives at the repo root (unlike the individual stage demos) so both
``python main.py`` and ``python -m main`` work without the
"ModuleNotFoundError: No module named 'stage1_dynamics_attitude'" issue you
get running a stageN script directly -- that error happens because Python
only puts a directly-run script's own folder on the import path, not the
repo root, so sibling stage packages can't be found. main.py doesn't have
that problem since it already lives at the repo root.

Usage:
    python main.py
    python main.py --force    # regenerate every plot even if it already exists
"""

import argparse
import importlib
import os

import matplotlib
matplotlib.use("Agg")  # headless-safe backend; must be set before any stage
                        # module imports matplotlib.pyplot for the first time

from stage1_dynamics_attitude.run_hover_demo import run as run_stage1
from stage2_trajectory_planning.run_trajectory_demo import run as run_stage2
from stage3_sensing_environment.run_sensor_demo import run_demo as run_stage3
from stage4_logging_integration.run_full_demo import run as run_stage4
from stage6_ardupilot_sim.run_ardupilot_demo import run as run_stage6


def _optional_stage(module_name: str):
    """
    Import a stage whose simulator is an optional install (PyBullet,
    Gazebo) -- returns (run_fn, None), or (None, reason) if its dependency
    is missing, so one absent simulator doesn't stop every other stage.
    """
    try:
        return importlib.import_module(module_name).run, None
    except ImportError as err:
        return None, str(err)


run_stage5, stage5_missing = _optional_stage("stage5_pybullet_sim.run_pybullet_demo")
run_stage7, stage7_missing = _optional_stage("stage7_gazebo_sim.run_gazebo_demo")

REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(REPO_ROOT, "outputs")

STAGES = [
    ("Stage 1 (dynamics & attitude)", "stage1_hover_tracking.png", run_stage1, None),
    ("Stage 2 (trajectory planning)", "stage2_trajectory_tracking.png", run_stage2, None),
    ("Stage 3 (sensing & environment)", "stage3_sensor_demo.png", run_stage3, None),
    ("Stage 4 (full integration)", "stage4_full_integration.png", run_stage4, None),
    ("Stage 5 (PyBullet simulation)", "stage5_pybullet_tracking.png", run_stage5, stage5_missing),
    ("Stage 6 (ArduPilot SITL integration)", "stage6_ardupilot_tracking.png", run_stage6, None),
    ("Stage 7 (Gazebo simulation)", "stage7_gazebo_tracking.png", run_stage7, stage7_missing),
]



def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--force", action="store_true",
                         help="regenerate every plot, even ones that already exist")
    args = parser.parse_args()

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    for label, filename, run_fn, missing_reason in STAGES:
        out_path = os.path.join(OUTPUT_DIR, filename)
        print(f"\n=== {label} ===")
        if run_fn is None:
            print(f"Skipping -- simulator not installed: {missing_reason}")
            continue
        if os.path.exists(out_path) and not args.force:
            print(f"Skipping -- {filename} already exists (use --force to regenerate).")
            continue
        run_fn()

    print(f"\nAll stages complete. Plots saved in: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
