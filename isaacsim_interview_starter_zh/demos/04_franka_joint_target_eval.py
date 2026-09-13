"""Franka 最小闭环任务：加载机器人、下发关节目标、评测是否到达。

从 Isaac Sim 安装根目录运行：
    .\\python.bat ..\\isaacsim_interview_starter_zh\\demos\\04_franka_joint_target_eval.py --seconds 8

这个例子刻意使用关节空间目标，而不是立即引入 IK。理解它之后，把
make_action() 替换为 IK、motion planner 或 RL policy，就能形成末端任务控制。
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

parser = argparse.ArgumentParser(description="Franka joint-space reach task with evaluation")
parser.add_argument("--headless", action="store_true", help="不打开 viewport")
parser.add_argument("--seconds", type=float, default=8.0, help="任务运行时长")
parser.add_argument("--success-threshold", type=float, default=0.08, help="7 个机械臂关节误差的 L2 阈值（rad）")
args, kit_args = parser.parse_known_args()
# SimulationApp 也会解析 sys.argv；保留只属于 Kit 的参数，避免自定义参数冲突。
sys.argv = [sys.argv[0], *kit_args]

# 必须是第一个 Isaac Sim import：它启动 Kit 与所需 extension。
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": args.headless})

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.timeline
from isaacsim.core.experimental.prims import Articulation
from isaacsim.storage.native import get_assets_root_path


# Franka: 7 个 arm DOF + 2 个 finger DOF，单位为 rad / m（手指关节）。
HOME = np.array([0.0, -0.75, 0.0, -2.20, 0.0, 1.55, 0.75, 0.04, 0.04], dtype=np.float32)
GOAL = np.array([0.20, -0.95, 0.15, -2.35, -0.10, 1.75, 0.65, 0.04, 0.04], dtype=np.float32)


def build_scene() -> Articulation:
    """创建 Stage、引用官方 Franka USD，并返回可读写的 articulation wrapper。"""
    assets_root = get_assets_root_path()
    if assets_root is None:
        raise RuntimeError("Assets root not found. Run post_install.bat from the Isaac Sim install directory.")

    stage_utils.create_new_stage(template="default stage")
    robot_prim = stage_utils.add_reference_to_stage(
        usd_path=assets_root + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
        path="/World/Franka",
        variants=[("Gripper", "AlternateFinger"), ("Mesh", "Performance")],
    )
    if not robot_prim or not robot_prim.IsValid():
        raise RuntimeError("Could not add Franka USD to /World/Franka")

    robot = Articulation("/World/Franka")
    robot.set_default_state(dof_positions=HOME.tolist())
    return robot


def make_action(elapsed: float) -> np.ndarray:
    """上层控制器接口：此处返回关节位置 action。

    采用 1 秒平滑插值，而不是从 HOME 瞬间跳到 GOAL，降低关节 drive
    的冲击。以后替换为 IK/policy 时，仍应保持 action 单位和关节顺序一致。
    """
    phase = np.clip(elapsed / 1.0, 0.0, 1.0)
    smooth_phase = phase * phase * (3.0 - 2.0 * phase)  # smoothstep
    return HOME + smooth_phase * (GOAL - HOME)


def evaluate(current_dofs: np.ndarray) -> tuple[float, bool]:
    """任务评测接口：只比较机械臂 7 个 DOF，不把夹爪保持量混入误差。"""
    arm_error = float(np.linalg.norm(current_dofs[:7] - GOAL[:7]))
    return arm_error, arm_error <= args.success_threshold


def main() -> int:
    try:
        robot = build_scene()
    except RuntimeError as exc:
        carb.log_error(str(exc))
        return 1

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    simulation_app.update()  # 初始化物理后端与 articulation 数据。
    robot.reset_to_default_state()

    print("Task: move Franka from HOME to a joint-space GOAL and evaluate final tracking error.")
    print(f"Success threshold: {args.success_threshold:.3f} rad (L2 over 7 arm joints)")

    elapsed = 0.0
    step = 0
    dt = 1.0 / 60.0
    final_error = float("inf")
    success = False
    while simulation_app.is_running() and elapsed < args.seconds:
        action = make_action(elapsed)
        # 这是控制代码接入点：将 action 交给 USD 机器人关节的 position drive。
        robot.set_dof_position_targets(action.tolist())
        simulation_app.update()

        current = robot.get_dof_positions().numpy()[0]
        final_error, success = evaluate(current)
        if step % 60 == 0:
            print(
                f"t={elapsed:4.1f}s | target_j1={action[0]:+.3f} | "
                f"actual_j1={current[0]:+.3f} | arm_error={final_error:.4f} rad"
            )
        elapsed += dt
        step += 1

    print(f"Evaluation: final_arm_error={final_error:.4f} rad, success={success}, steps={step}")
    timeline.stop()
    return 0 if success else 2


try:
    sys.exit(main())
finally:
    simulation_app.close()
