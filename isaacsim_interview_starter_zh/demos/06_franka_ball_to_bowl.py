"""PhysX pick-and-place demo: Franka moves a dynamic ball into a physical bowl.

Run from the Isaac Sim installation root:
    .\\python.bat ..\\isaacsim_interview_starter_zh\\demos\\06_franka_ball_to_bowl.py

The controller is NVIDIA's bundled PickPlaceController.  The ball and the
bowl's four walls are real PhysX colliders; the bowl bottom is the ground plane.
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

parser = argparse.ArgumentParser(description="Franka: pick a ball and place it in a bowl")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--max-steps", type=int, default=0, help="0 keeps the viewer open until it is closed.")
args, kit_args = parser.parse_known_args()
# SimulationApp forwards ``sys.argv`` to Kit.  Keep only genuine Kit options:
# Kit has its own --max-steps option, which otherwise shuts the app down after
# one frame before this demo can execute its Python control loop.
sys.argv = [sys.argv[0], *kit_args]

from isaacsim import SimulationApp


simulation_app = SimulationApp(
    {"headless": args.headless, "extra_args": ["--enable", "isaacsim.robot.manipulators.examples"]}
)

import carb
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicSphere, FixedCuboid
from isaacsim.core.api.materials import PhysicsMaterial
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.examples.franka.controllers.pick_place_controller import PickPlaceController
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, UsdPhysics


BALL_RADIUS = 0.025
BALL_START = np.array([0.35, 0.25, BALL_RADIUS])
BOWL_CENTER = np.array([-0.30, -0.30, 0.0])
GRASP_JOINT_PATH = "/World/GraspConstraint"
END_EFFECTOR_PATH = "/World/Franka/panda_rightfinger"


def add_physical_bowl(world: World) -> None:
    """Build an open square bowl from fixed PhysX walls over the ground plane."""
    wall_height = 0.09
    half_inner_width = 0.10
    wall_thickness = 0.018
    wall_color = np.array([0.15, 0.55, 0.95])
    specs = [
        ("front", [0.0, -half_inner_width - wall_thickness / 2, wall_height / 2], [0.24, wall_thickness, wall_height]),
        ("back", [0.0, half_inner_width + wall_thickness / 2, wall_height / 2], [0.24, wall_thickness, wall_height]),
        ("left", [-half_inner_width - wall_thickness / 2, 0.0, wall_height / 2], [wall_thickness, 0.24, wall_height]),
        ("right", [half_inner_width + wall_thickness / 2, 0.0, wall_height / 2], [wall_thickness, 0.24, wall_height]),
    ]
    for label, offset, scale in specs:
        world.scene.add(
            FixedCuboid(
                prim_path=f"/World/Bowl/{label}",
                name=f"bowl_{label}",
                position=BOWL_CENTER + np.array(offset),
                scale=np.array(scale),
                size=1.0,
                color=wall_color,
            )
        )


def create_grasp_constraint(stage, finger_position, finger_orientation, ball_position, ball_orientation) -> None:
    """Attach the ball to the closing finger without changing its world pose.

    This is a task-level grasp abstraction: contact and high friction make the
    approach look physical, then a fixed PhysX joint represents a successful
    grasp until the controller opens over the bowl.
    """
    stage.RemovePrim(GRASP_JOINT_PATH)
    finger_quat = Gf.Quatf(
        float(finger_orientation[0]),
        Gf.Vec3f(float(finger_orientation[1]), float(finger_orientation[2]), float(finger_orientation[3])),
    )
    ball_quat = Gf.Quatf(
        float(ball_orientation[0]),
        Gf.Vec3f(float(ball_orientation[1]), float(ball_orientation[2]), float(ball_orientation[3])),
    )
    offset_world = Gf.Vec3f(*(ball_position - finger_position).astype(float))
    offset_in_finger = finger_quat.GetInverse().Transform(offset_world)

    joint = UsdPhysics.FixedJoint.Define(stage, GRASP_JOINT_PATH)
    joint.CreateBody0Rel().SetTargets([END_EFFECTOR_PATH])
    joint.CreateBody1Rel().SetTargets(["/World/Ball"])
    joint.CreateLocalPos0Attr().Set(offset_in_finger)
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
    joint.CreateLocalRot1Attr().Set(ball_quat.GetInverse() * finger_quat)
    # The ball must remain a separate rigid body, rather than becoming a link
    # of Franka's articulation.
    joint.CreateExcludeFromArticulationAttr().Set(True)
    joint.CreateBreakForceAttr().Set(1.0e6)
    joint.CreateBreakTorqueAttr().Set(1.0e6)


def main() -> int:
    assets_root = get_assets_root_path()
    if assets_root is None:
        carb.log_error("Could not find the Isaac Sim asset root")
        return 1

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    add_physical_bowl(world)

    robot_prim = add_reference_to_stage(
        assets_root + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd", "/World/Franka"
    )
    robot_prim.GetVariantSet("Gripper").SetVariantSelection("AlternateFinger")
    robot_prim.GetVariantSet("Mesh").SetVariantSelection("Quality")
    gripper = ParallelGripper(
        end_effector_prim_path="/World/Franka/panda_rightfinger",
        joint_prim_names=["panda_finger_joint1", "panda_finger_joint2"],
        joint_opened_positions=np.array([0.05, 0.05]),
        joint_closed_positions=np.array([0.018, 0.018]),
        action_deltas=np.array([0.01, 0.01]),
    )
    franka = world.scene.add(
        SingleManipulator(
            prim_path="/World/Franka",
            name="franka",
            end_effector_prim_path="/World/Franka/panda_rightfinger",
            gripper=gripper,
        )
    )
    ball_material = PhysicsMaterial(
        prim_path="/World/Materials/BallGripMaterial",
        static_friction=1.2,
        dynamic_friction=1.0,
        restitution=0.0,
    )
    ball = world.scene.add(
        DynamicSphere(
            prim_path="/World/Ball",
            name="ball",
            radius=BALL_RADIUS,
            position=BALL_START,
            mass=0.02,
            color=np.array([0.95, 0.10, 0.10]),
            physics_material=ball_material,
        )
    )

    gripper.set_default_state(gripper.joint_opened_positions)
    world.reset()
    controller = PickPlaceController(name="ball_to_bowl", gripper=gripper, robot_articulation=franka)
    articulation_controller = franka.get_articulation_controller()
    stage = omni.usd.get_context().get_stage()

    completed = False
    grasped = False
    released = False
    grasp_checked = False
    step = 0
    # In headless Kit sessions ``is_running()`` may become false even though a
    # caller explicitly requested a fixed number of simulation steps.  Keep
    # the interactive behaviour for the GUI, while letting --max-steps be the
    # authoritative stop condition for repeatable batch tests.
    while (args.max_steps > 0 and step < args.max_steps) or (
        args.max_steps <= 0 and simulation_app.is_running()
    ):
        world.step(render=not args.headless)
        if world.is_playing():
            ball_position = ball.get_local_pose()[0]
            action = controller.forward(
                picking_position=ball_position,
                placing_position=BOWL_CENTER + np.array([0.0, 0.0, BALL_RADIUS]),
                current_joint_positions=franka.get_joint_positions(),
                end_effector_offset=np.array([0.0, 0.005, 0.0]),
            )
            articulation_controller.apply_action(action)
            event = controller.get_current_event()
            if event >= 4 and not grasped and not grasp_checked:
                grasp_checked = True
                finger_position, finger_orientation = franka.end_effector.get_world_pose()
                ball_position, ball_orientation = ball.get_world_pose()
                distance = np.linalg.norm(ball_position - finger_position)
                if distance < 0.075:
                    create_grasp_constraint(
                        stage, finger_position, finger_orientation, ball_position, ball_orientation
                    )
                    grasped = True
                    print(f"Grasp attached: finger-to-ball distance={distance * 100:.1f} cm", flush=True)
                else:
                    print(f"Grasp missed: finger-to-ball distance={distance * 100:.1f} cm", flush=True)
            if grasped and not released and event >= 8:
                stage.RemovePrim(GRASP_JOINT_PATH)
                released = True
                print("Ball released above bowl", flush=True)
            if controller.is_done() and not completed:
                horizontal_error = np.linalg.norm(ball_position[:2] - BOWL_CENTER[:2])
                in_bowl = horizontal_error < 0.09 and ball_position[2] < 0.10
                print(f"Task finished: ball={ball_position}, in_bowl={in_bowl}")
                completed = True
        step += 1

    if args.max_steps > 0:
        message = f"Simulation completed: {step}/{args.max_steps} steps"
        carb.log_info(message)
        print(message, flush=True)

    return 0


try:
    sys.exit(main())
finally:
    simulation_app.close()
