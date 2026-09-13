"""A visual Franka Panda differential-IK target-tracking demo for Isaac Sim 6.

The scene uses NVIDIA's bundled Franka Panda USD asset (Apache-2.0) and the
Isaac Sim experimental API.  A damped least-squares differential IK controller
moves the end effector toward a visible sphere or cube target.

Run from the Isaac Sim installation directory:
    .\\python.bat ..\\isaacsim_interview_starter_zh\\demos\\05_franka_wave_demo.py

Use --seconds 20 to stop automatically after 20 seconds, or --headless for a
non-visual smoke test.
"""

from __future__ import annotations

import argparse
import sys

import numpy as np


parser = argparse.ArgumentParser(description="Visual Franka Panda target-tracking demo")
parser.add_argument(
    "--seconds",
    type=float,
    default=0.0,
    help="Seconds to run; 0 (the default) keeps running until the Isaac Sim window is closed.",
)
parser.add_argument("--headless", action="store_true", help="Run without an Isaac Sim viewport.")
parser.add_argument(
    "--target-shape", choices=["sphere", "cube"], default="sphere", help="Visual target primitive."
)
parser.add_argument(
    "--success-threshold",
    type=float,
    default=0.02,
    help="End-effector distance threshold in metres used by the reported success metric (default: 0.02).",
)
args, _ = parser.parse_known_args()

# SimulationApp must be created before importing Omniverse/Isaac Sim APIs.
from isaacsim import SimulationApp


simulation_app = SimulationApp({"headless": args.headless})

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.timeline
from isaacsim.core.experimental.materials import PreviewSurfaceMaterial, RigidBodyMaterial
from isaacsim.core.experimental.objects import Cube, Sphere
from isaacsim.core.experimental.prims import Articulation, GeomPrim, RigidPrim
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.storage.native import get_assets_root_path


HOME_POSE = np.array([0.0, -1.3, 0.0, -2.87, 0.0, 2.0, 0.75, 0.04, 0.04], dtype=np.float32)
OBJECT_START_POSITION = np.array([0.55, 0.0, 0.30], dtype=np.float32)
OBJECT_SIZE = 0.10


def damped_least_squares_delta(jacobian_position: np.ndarray, position_error: np.ndarray) -> np.ndarray:
    """Compute a 7-DOF joint update from a 3x7 position Jacobian.

    This is differential inverse kinematics: dq = J.T (J J.T + lambda^2 I)^-1 dx.
    Damping avoids unstable updates near singular configurations.
    """
    damping = 0.06
    identity = np.eye(3, dtype=np.float32)
    return jacobian_position.T @ np.linalg.solve(
        jacobian_position @ jacobian_position.T + (damping**2) * identity,
        position_error,
    )


def main() -> None:
    assets_root = get_assets_root_path()
    if assets_root is None:
        carb.log_error("Isaac Sim assets were not found. Run post_install.bat, then try again.")
        simulation_app.close()
        sys.exit(1)

    # The default-stage template supplies lighting and a ground plane.
    stage_utils.create_new_stage(template="default stage")
    robot_prim = stage_utils.add_reference_to_stage(
        usd_path=assets_root + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
        path="/World/Franka",
        variants=[("Gripper", "AlternateFinger"), ("Mesh", "Performance")],
    )
    if not robot_prim or not robot_prim.IsValid():
        raise RuntimeError("Could not add the Franka Panda USD reference to /World/Franka")

    # Create a *dynamic* PhysX target: it has collision, mass, gravity, and
    # friction.  It is not teleported every frame; physics determines its pose.
    red = PreviewSurfaceMaterial("/World/Looks/RedTarget")
    red.set_input_values("diffuseColor", [0.95, 0.05, 0.05])
    target_marker = (
        Sphere(
            "/World/TargetMarker",
            radii=[OBJECT_SIZE / 2.0],
            positions=OBJECT_START_POSITION,
            reset_xform_op_properties=True,
        )
        if args.target_shape == "sphere"
        else Cube(
            "/World/TargetMarker",
            sizes=OBJECT_SIZE,
            positions=OBJECT_START_POSITION,
            reset_xform_op_properties=True,
        )
    )
    target_marker.apply_visual_materials(red)
    target_collision = GeomPrim("/World/TargetMarker", apply_collision_apis=True)
    target_collision.set_collision_approximations("sphereFill" if args.target_shape == "sphere" else "convexHull")
    target_material = RigidBodyMaterial(
        "/World/Looks/TargetPhysics", static_frictions=0.9, dynamic_frictions=0.7, restitutions=0.0
    )
    target_collision.apply_physics_materials(target_material)
    target_rigid_body = RigidPrim("/World/TargetMarker", masses=[0.05])

    if not args.headless:
        ViewportManager.set_camera_view(
            "/OmniverseKit_Persp", eye=[2.2, 2.2, 1.45], target=[0.0, 0.0, 0.55]
        )

    robot = Articulation("/World/Franka")
    end_effector = RigidPrim("/World/Franka/panda_hand")
    end_effector_link_index = robot.get_link_indices("panda_hand").list()[0]
    robot.set_default_state(dof_positions=HOME_POSE.tolist())
    robot.set_enabled_self_collisions([True])

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    simulation_app.update()  # Allow physics and the articulation wrapper to initialize.
    robot.reset_to_default_state()
    # Let the 50 g target fall onto and settle on the default PhysX ground plane.
    for _ in range(120):
        simulation_app.update()

    print("Franka Panda PhysX collision + differential-IK pre-grasp demo is running.")
    print("Close the Isaac Sim window to stop, or use --seconds N for a timed run.")
    elapsed = 0.0
    dt = 1.0 / 60.0
    tracking_errors: list[float] = []
    while simulation_app.is_running() and (args.seconds <= 0.0 or elapsed < args.seconds):
        # Read the physical object's simulated pose.  Aim above its upper
        # surface rather than at its centre, so this is a collision-safe
        # pre-grasp target instead of an intentional visual intersection.
        object_position = target_rigid_body.get_world_poses()[0].numpy()[0]
        marker_position = object_position + np.array([0.0, 0.0, OBJECT_SIZE / 2.0 + 0.10], dtype=np.float32)

        # Read state -> calculate DLS differential IK -> send the seven arm targets.
        current_joint_positions = robot.get_dof_positions().numpy()[0]
        end_effector_position = end_effector.get_world_poses()[0].numpy()[0]
        position_error = marker_position - end_effector_position
        error_metres = float(np.linalg.norm(position_error))
        tracking_errors.append(error_metres)
        jacobians = robot.get_jacobian_matrices().numpy()
        position_jacobian = jacobians[0, end_effector_link_index - 1, :3, :7]
        joint_delta = damped_least_squares_delta(position_jacobian, position_error)
        arm_position_target = current_joint_positions[:7] + 0.65 * joint_delta
        robot.set_dof_position_targets(arm_position_target.tolist(), dof_indices=list(range(7)))

        # Keep the gripper open; grasping can be added as a separate task state.
        robot.set_dof_position_targets([0.04, 0.04], dof_indices=[7, 8])
        simulation_app.update()
        if len(tracking_errors) % 60 == 0:
            print(f"tracking error: {error_metres * 100.0:.1f} cm")
        elapsed += dt

    if tracking_errors:
        final_error = tracking_errors[-1]
        print(
            f"Tracking summary: final={final_error * 100.0:.2f} cm, "
            f"mean={np.mean(tracking_errors) * 100.0:.2f} cm, "
            f"success(final <= {args.success_threshold * 100.0:.1f} cm)="
            f"{final_error <= args.success_threshold}"
        )
    timeline.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
