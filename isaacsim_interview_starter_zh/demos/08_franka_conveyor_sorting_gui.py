"""Native Isaac Sim GUI demo: a Franka sorts two workpieces from a conveyor.

This is intentionally a Kit/USD/PhysX demo, not a Newton Viewer example.
The robot, conveyor, bins, rigid bodies and fixed grasp constraints all live
on the Isaac Sim USD Stage, so their motion appears in the Isaac Sim viewport.

Run from the Isaac Sim installation root:
    .\\python.bat ..\\isaacsim_interview_starter_zh\\demos\\08_franka_conveyor_sorting_gui.py

Controls:
    - Use the normal Isaac Sim viewport camera controls to inspect the cell.
    - The terminal prints the task state and final sorting metrics.
    - ``--max-steps 0`` keeps the GUI open until you close it.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass

import numpy as np

parser = argparse.ArgumentParser(description="Native Isaac Sim Franka conveyor sorting demo")
parser.add_argument("--headless", action="store_true", help="Run without the Isaac Sim viewport")
parser.add_argument(
    "--max-steps",
    type=int,
    default=2400,
    help="Maximum simulation steps; 0 keeps the GUI open after the task.",
)
args, kit_args = parser.parse_known_args()
# Do not let Kit consume this demo's --max-steps argument.
sys.argv = [sys.argv[0], *kit_args]

from isaacsim import SimulationApp


simulation_app = SimulationApp(
    {
        "headless": args.headless,
        "extra_args": ["--enable", "isaacsim.robot.manipulators.examples"],
    }
)

import carb
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.materials import PhysicsMaterial
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.examples.franka.controllers.pick_place_controller import PickPlaceController
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, UsdPhysics


END_EFFECTOR_PATH = "/World/Franka/panda_rightfinger"
GRASP_JOINT_PATH = "/World/SortingCell/ActiveGraspConstraint"
CUBE_SIZE = 0.050
CUBE_HALF_HEIGHT = CUBE_SIZE / 2.0


@dataclass(frozen=True)
class SortingJob:
    """The controller contract for one workpiece: object state -> bin action."""

    label: str
    object_name: str
    object_path: str
    target_bin: np.ndarray
    color: np.ndarray


def add_bin(world: World, label: str, center: np.ndarray, color: np.ndarray) -> None:
    """Create a physical open bin from four fixed colliders on the USD stage."""
    wall_height = 0.11
    inner_half_width = 0.105
    thickness = 0.016
    specs = [
        ("front", [0.0, -inner_half_width - thickness / 2.0, wall_height / 2.0], [0.25, thickness, wall_height]),
        ("back", [0.0, inner_half_width + thickness / 2.0, wall_height / 2.0], [0.25, thickness, wall_height]),
        ("left", [-inner_half_width - thickness / 2.0, 0.0, wall_height / 2.0], [thickness, 0.25, wall_height]),
        ("right", [inner_half_width + thickness / 2.0, 0.0, wall_height / 2.0], [thickness, 0.25, wall_height]),
    ]
    for wall_name, offset, scale in specs:
        world.scene.add(
            FixedCuboid(
                prim_path=f"/World/SortingCell/{label}Bin/{wall_name}",
                name=f"{label}_{wall_name}",
                position=center + np.array(offset),
                scale=np.array(scale),
                size=1.0,
                color=color,
            )
        )


def add_sorting_cell(world: World) -> None:
    """Build visual and physical infrastructure: conveyor, rails and two bins."""
    dark = np.array([0.08, 0.10, 0.13])
    metal = np.array([0.35, 0.38, 0.42])

    world.scene.add(
        FixedCuboid(
            prim_path="/World/SortingCell/ConveyorBase",
            name="conveyor_base",
            position=np.array([0.48, 0.10, 0.065]),
            scale=np.array([0.62, 0.42, 0.13]),
            size=1.0,
            color=dark,
        )
    )
    world.scene.add(
        FixedCuboid(
            prim_path="/World/SortingCell/ConveyorBelt",
            name="conveyor_belt",
            position=np.array([0.48, 0.10, 0.135]),
            scale=np.array([0.58, 0.36, 0.018]),
            size=1.0,
            color=np.array([0.12, 0.16, 0.19]),
        )
    )
    for side, y in (("left", -0.105), ("right", 0.305)):
        world.scene.add(
            FixedCuboid(
                prim_path=f"/World/SortingCell/ConveyorRail/{side}",
                name=f"conveyor_rail_{side}",
                position=np.array([0.48, y, 0.20]),
                scale=np.array([0.60, 0.018, 0.14]),
                size=1.0,
                color=metal,
            )
        )

    add_bin(world, "Red", np.array([-0.30, -0.30, 0.0]), np.array([0.92, 0.12, 0.12]))
    add_bin(world, "Blue", np.array([-0.30, 0.32, 0.0]), np.array([0.12, 0.35, 0.95]))


def create_grasp_constraint(
    stage,
    object_path: str,
    finger_position: np.ndarray,
    finger_orientation: np.ndarray,
    object_position: np.ndarray,
    object_orientation: np.ndarray,
) -> None:
    """Create a fixed PhysX joint only after a verified near-grasp.

    The cubes remain separate rigid bodies.  The fixed joint is a deliberate
    task-level abstraction for a reliable gripper grasp; it is released over
    the target bin and is not presented as a contact-only grasp model.
    """
    stage.RemovePrim(GRASP_JOINT_PATH)
    finger_quat = Gf.Quatf(
        float(finger_orientation[0]),
        Gf.Vec3f(float(finger_orientation[1]), float(finger_orientation[2]), float(finger_orientation[3])),
    )
    object_quat = Gf.Quatf(
        float(object_orientation[0]),
        Gf.Vec3f(float(object_orientation[1]), float(object_orientation[2]), float(object_orientation[3])),
    )
    object_offset_world = Gf.Vec3f(*(object_position - finger_position).astype(float))
    object_offset_in_finger = finger_quat.GetInverse().Transform(object_offset_world)

    joint = UsdPhysics.FixedJoint.Define(stage, GRASP_JOINT_PATH)
    joint.CreateBody0Rel().SetTargets([END_EFFECTOR_PATH])
    joint.CreateBody1Rel().SetTargets([object_path])
    joint.CreateLocalPos0Attr().Set(object_offset_in_finger)
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
    joint.CreateLocalRot1Attr().Set(object_quat.GetInverse() * finger_quat)
    joint.CreateExcludeFromArticulationAttr().Set(True)
    joint.CreateBreakForceAttr().Set(1.0e6)
    joint.CreateBreakTorqueAttr().Set(1.0e6)


def in_target_bin(position: np.ndarray, target_bin: np.ndarray) -> bool:
    """A reproducible task metric, independent from the viewport appearance."""
    horizontal_error = float(np.linalg.norm(position[:2] - target_bin[:2]))
    return horizontal_error < 0.09 and position[2] < 0.12


def main() -> int:
    assets_root = get_assets_root_path()
    if assets_root is None:
        carb.log_error("Assets root not found. Run post_install.bat from the Isaac Sim installation directory.")
        return 1

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    add_sorting_cell(world)

    robot_prim = add_reference_to_stage(
        assets_root + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd", "/World/Franka"
    )
    robot_prim.GetVariantSet("Gripper").SetVariantSelection("AlternateFinger")
    robot_prim.GetVariantSet("Mesh").SetVariantSelection("Quality")
    gripper = ParallelGripper(
        end_effector_prim_path=END_EFFECTOR_PATH,
        joint_prim_names=["panda_finger_joint1", "panda_finger_joint2"],
        joint_opened_positions=np.array([0.05, 0.05]),
        joint_closed_positions=np.array([0.018, 0.018]),
        action_deltas=np.array([0.01, 0.01]),
    )
    franka = world.scene.add(
        SingleManipulator(
            prim_path="/World/Franka",
            name="franka",
            end_effector_prim_path=END_EFFECTOR_PATH,
            gripper=gripper,
        )
    )

    cube_material = PhysicsMaterial(
        prim_path="/World/SortingCell/Materials/HighGrip",
        static_friction=1.1,
        dynamic_friction=0.9,
        restitution=0.0,
    )
    jobs = [
        SortingJob(
            label="red-to-red-bin",
            object_name="red_cube",
            object_path="/World/SortingCell/Workpieces/RedCube",
            target_bin=np.array([-0.30, -0.30, CUBE_HALF_HEIGHT]),
            color=np.array([0.95, 0.08, 0.08]),
        ),
        SortingJob(
            label="blue-to-blue-bin",
            object_name="blue_cube",
            object_path="/World/SortingCell/Workpieces/BlueCube",
            target_bin=np.array([-0.30, 0.32, CUBE_HALF_HEIGHT]),
            color=np.array([0.08, 0.30, 0.95]),
        ),
    ]
    workpieces = {}
    # Keep both parts in the verified Franka reach workspace.  The second
    # workpiece is separated along Y so the first completed pick cannot push it.
    start_positions = [np.array([0.39, 0.00, 0.17]), np.array([0.37, 0.20, 0.17])]
    for job, position in zip(jobs, start_positions, strict=True):
        workpieces[job.object_name] = world.scene.add(
            DynamicCuboid(
                prim_path=job.object_path,
                name=job.object_name,
                position=position,
                scale=np.array([CUBE_SIZE, CUBE_SIZE, CUBE_SIZE]),
                size=1.0,
                mass=0.035,
                color=job.color,
                physics_material=cube_material,
            )
        )

    gripper.set_default_state(gripper.joint_opened_positions)
    world.reset()
    stage = omni.usd.get_context().get_stage()
    articulation_controller = franka.get_articulation_controller()

    job_index = 0
    controller = PickPlaceController(name="sort_0", gripper=gripper, robot_articulation=franka)
    grasped = False
    released = False
    grasp_checked = False
    completed = []
    step = 0
    print("Native Isaac Sim sorting cell started: red -> red bin, blue -> blue bin.", flush=True)

    while (args.max_steps > 0 and step < args.max_steps) or (
        args.max_steps <= 0 and simulation_app.is_running()
    ):
        world.step(render=not args.headless)
        if not world.is_playing():
            step += 1
            continue

        if job_index < len(jobs):
            job = jobs[job_index]
            workpiece = workpieces[job.object_name]
            object_position = workpiece.get_world_pose()[0]
            action = controller.forward(
                picking_position=object_position,
                placing_position=job.target_bin,
                current_joint_positions=franka.get_joint_positions(),
                end_effector_offset=np.array([0.0, 0.005, 0.0]),
            )
            articulation_controller.apply_action(action)
            event = controller.get_current_event()

            if event >= 4 and not grasped and not grasp_checked:
                grasp_checked = True
                finger_position, finger_orientation = franka.end_effector.get_world_pose()
                object_position, object_orientation = workpiece.get_world_pose()
                distance = float(np.linalg.norm(object_position - finger_position))
                if distance < 0.080:
                    create_grasp_constraint(
                        stage,
                        job.object_path,
                        finger_position,
                        finger_orientation,
                        object_position,
                        object_orientation,
                    )
                    grasped = True
                    print(f"[{job.label}] grasp attached, distance={distance * 100:.1f} cm", flush=True)
                else:
                    print(f"[{job.label}] grasp missed, distance={distance * 100:.1f} cm", flush=True)

            if grasped and not released and event >= 8:
                stage.RemovePrim(GRASP_JOINT_PATH)
                released = True
                print(f"[{job.label}] released over target bin", flush=True)

            if controller.is_done():
                object_position = workpiece.get_world_pose()[0]
                success = in_target_bin(object_position, job.target_bin)
                completed.append(success)
                print(
                    f"[{job.label}] complete: object={np.round(object_position, 3)}, "
                    f"in_target_bin={success}",
                    flush=True,
                )
                job_index += 1
                if job_index < len(jobs):
                    controller = PickPlaceController(
                        name=f"sort_{job_index}", gripper=gripper, robot_articulation=franka
                    )
                    grasped = False
                    released = False
                    grasp_checked = False

        elif step % 240 == 0:
            print(f"All sorting jobs complete: {sum(completed)}/{len(jobs)} successful. Inspect the USD stage.", flush=True)

        step += 1

    stage.RemovePrim(GRASP_JOINT_PATH)
    success_count = sum(completed)
    print(
        f"Sorting evaluation: completed={len(completed)}/{len(jobs)}, "
        f"success={success_count}/{len(jobs)}, steps={step}",
        flush=True,
    )
    return 0 if len(completed) == len(jobs) and success_count == len(jobs) else 2


try:
    sys.exit(main())
finally:
    simulation_app.close()
