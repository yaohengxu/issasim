"""Launch the bundled GPU robot-cloth manipulation demo.

This delegates to Isaac Sim 6's bundled NVIDIA Newton example rather than
reimplementing a soft-body solver.  It simulates a Franka arm and a deformable
cloth mesh with the VBD solver, including robot-cloth contact.

Run from the Isaac Sim installation root:
    .\\python.bat ..\\isaacsim_interview_starter_zh\\demos\\07_franka_soft_cloth_newton_demo.py

Newton has its own interactive viewer and may download its public sample assets
on the first launch.  All command-line options are forwarded to the example.
"""

from __future__ import annotations

import runpy
import sys
from pathlib import Path

# ``warp`` is supplied by Isaac Sim's ``omni.warp.core`` extension, not by
# Newton's ordinary pip bundle.  Starting SimulationApp first activates that
# extension and also registers the bundled ``isaacsim.pip.newton`` package.
# Newton then opens its own GLFW viewer; keep Kit headless to avoid a second
# Isaac Sim viewport window.
from isaacsim import SimulationApp


# This file lives in <workspace>/isaacsim_interview_starter_zh/demos, while the
# bundled Newton example lives inside the sibling Isaac Sim installation.
workspace_root = Path(__file__).resolve().parents[2]
root = workspace_root / "isaac-sim-standalone-6.0.1-windows-x86_64"
example = root / "exts" / "isaacsim.pip.newton" / "pip_prebundle" / "newton" / "examples" / "cloth" / "example_cloth_franka.py"
if not example.is_file():
    raise FileNotFoundError(
        "Bundled Newton cloth example not found. Expected: " + str(example)
    )

# Kit also reads ``sys.argv`` while SimulationApp starts.  Save Newton's flags
# and temporarily remove them, otherwise e.g. ``--help`` is consumed by Kit.
example_args = sys.argv[1:]
sys.argv = [sys.argv[0]]

# The ``finally`` also releases the GPU cleanly when argparse exits for --help.
simulation_app = SimulationApp({"headless": True})
try:
    sys.argv = [str(example), *example_args]
    runpy.run_path(str(example), run_name="__main__")
finally:
    simulation_app.close()
