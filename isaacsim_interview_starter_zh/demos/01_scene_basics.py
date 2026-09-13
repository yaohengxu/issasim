"""最小 PhysX 场景：地面、动态球、固定障碍物。

从 Isaac Sim 安装根目录运行：
    .\\python.bat ..\\isaacsim_interview_starter_zh\\demos\\01_scene_basics.py
    .\\python.bat ..\\isaacsim_interview_starter_zh\\demos\\01_scene_basics.py --headless --max-steps 240
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

parser = argparse.ArgumentParser(description="Isaac Sim 最小 PhysX 场景")
parser.add_argument("--headless", action="store_true", help="不打开 viewport")
parser.add_argument("--max-steps", type=int, default=0, help="0 表示 GUI 关闭前一直运行")
args, kit_args = parser.parse_known_args()
sys.argv = [sys.argv[0], *kit_args]

# 这一行必须发生在任何 isaacsim.core / omni import 之前。
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": args.headless})

from isaacsim.core.api import World
from isaacsim.core.api.materials import PhysicsMaterial
from isaacsim.core.api.objects import DynamicSphere, FixedCuboid


def main() -> None:
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # FixedCuboid 有可见网格和碰撞体，但不受重力影响：它是斜坡障碍物。
    world.scene.add(
        FixedCuboid(
            prim_path="/World/Ramp",
            name="ramp",
            position=np.array([0.25, 0.0, 0.12]),
            scale=np.array([0.50, 0.30, 0.10]),
            color=np.array([0.20, 0.45, 0.85]),
        )
    )
    ball_material = PhysicsMaterial(
        prim_path="/World/Materials/BallMaterial", static_friction=0.8, dynamic_friction=0.6, restitution=0.15
    )
    ball = world.scene.add(
        DynamicSphere(
            prim_path="/World/Ball",
            name="ball",
            radius=0.05,
            position=np.array([-0.35, 0.0, 0.75]),
            mass=0.05,
            color=np.array([0.95, 0.10, 0.10]),
            physics_material=ball_material,
        )
    )

    # reset 后物理句柄才完成初始化；所有 episode 都应有明确 reset。
    world.reset()
    print("Scene ready: red dynamic ball will fall and collide with ground/ramp.")

    step = 0
    while (args.max_steps > 0 and step < args.max_steps) or (
        args.max_steps <= 0 and simulation_app.is_running()
    ):
        world.step(render=not args.headless)
        if step % 60 == 0:
            position, _ = ball.get_world_pose()
            print(f"step={step:4d}, ball_position={np.round(position, 3)}")
        step += 1

    print(f"Finished after {step} steps.")


try:
    main()
finally:
    simulation_app.close()
