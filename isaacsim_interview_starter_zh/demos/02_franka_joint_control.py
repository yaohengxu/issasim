"""Franka Panda 关节位置控制：从 joint target 到 articulation motion。

从 Isaac Sim 安装根目录运行：
    .\\python.bat ..\\isaacsim_interview_starter_zh\\demos\\02_franka_joint_control.py --seconds 15

这个示例故意不做 IK：它展示底层接口的最小闭环。末端任务通常由
FK/IK 或 motion planner 先转换为关节目标，再通过相同的接口下发。
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

parser = argparse.ArgumentParser(description="Franka joint position control")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--seconds", type=float, default=0.0, help="0 表示直到窗口关闭")
args, kit_args = parser.parse_known_args()
sys.argv = [sys.argv[0], *kit_args]

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": args.headless})

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.timeline
from isaacsim.core.experimental.prims import Articulation
from isaacsim.storage.native import get_assets_root_path

HOME = np.array([0.0, -0.75, 0.0, -2.20, 0.0, 1.55, 0.75, 0.04, 0.04])


def main() -> int:
    assets_root = get_assets_root_path()
    if assets_root is None:
        carb.log_error("Assets root not found. Run post_install.bat in the Isaac Sim installation directory.")
        return 1

    # 与本机 6.0.1 的官方示例一致：先创建默认场景，再引用机器人 USD。
    stage_utils.create_new_stage(template="default stage")
    robot_prim = stage_utils.add_reference_to_stage(
        usd_path=assets_root + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
        path="/World/Franka",
        variants=[("Gripper", "AlternateFinger"), ("Mesh", "Performance")],
    )
    if not robot_prim or not robot_prim.IsValid():
        carb.log_error("Could not load Franka USD reference.")
        return 1
    franka = Articulation("/World/Franka")
    franka.set_default_state(dof_positions=HOME.tolist())

    # 启动物理时间线并让 articulation 句柄初始化；这与 World.reset 的目的相同。
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    simulation_app.update()
    franka.reset_to_default_state()
    print("Franka initialized. Sending smooth targets to joint 4; gripper stays open.")

    dt = 1.0 / 60.0
    elapsed = 0.0
    step = 0
    while simulation_app.is_running() and (args.seconds <= 0.0 or elapsed < args.seconds):
        target = HOME.copy()
        target[3] += 0.35 * np.sin(0.8 * elapsed)  # 第 4 个 arm joint 做平滑摆动
        target[7:] = 0.04  # 两个手指保持张开
        # 位置目标被 USD 内关节 drive（常可理解为底层 PD）执行。
        franka.set_dof_position_targets(target.tolist())
        simulation_app.update()

        if step % 60 == 0:
            current = franka.get_dof_positions().numpy()[0]
            print(f"t={elapsed:5.1f}s target_j4={target[3]:+.3f}, actual_j4={current[3]:+.3f}")
        elapsed += dt
        step += 1

    timeline.stop()
    return 0


try:
    sys.exit(main())
finally:
    simulation_app.close()
