"""无需启动 Isaac Sim 的 DLS differential IK 数学演示。

运行：
    .\\python.bat ..\\isaacsim_interview_starter_zh\\demos\\03_differential_ik_math.py

真实场景中 J 应从 articulation 的 Jacobian 读取；这里使用小矩阵，使
“末端误差 dx 如何变成关节增量 dq”可以直接观察。
"""

from __future__ import annotations

import numpy as np


def dls_ik(jacobian: np.ndarray, position_error: np.ndarray, damping: float = 0.08) -> np.ndarray:
    """dq = J.T @ inv(J @ J.T + lambda^2 I) @ dx."""
    task_dim = jacobian.shape[0]
    regularized = jacobian @ jacobian.T + damping**2 * np.eye(task_dim)
    return jacobian.T @ np.linalg.solve(regularized, position_error)


def main() -> None:
    # 一个 3D 位置任务、7 自由度机械臂的示意 Jacobian；不是某台真实机器人的参数。
    jacobian = np.array(
        [[0.42, -0.10, 0.25, 0.08, 0.00, 0.10, -0.05],
         [0.05,  0.38, 0.12, -0.20, 0.18, 0.00,  0.10],
         [0.10,  0.06, 0.32, 0.15, 0.10, -0.12, 0.22]],
        dtype=float,
    )
    dx = np.array([0.03, -0.02, 0.04])  # 希望末端移动的米数
    dq = dls_ik(jacobian, dx)
    achieved_dx = jacobian @ dq

    print("Desired end-effector delta (m): ", np.round(dx, 5))
    print("Joint delta (rad, illustrative): ", np.round(dq, 5))
    print("Linearized achieved delta (m):   ", np.round(achieved_dx, 5))
    print("Residual norm (m):", f"{np.linalg.norm(dx - achieved_dx):.6f}")
    print("Tip: increase damping near singularities; clamp dq before sending it to a robot.")


if __name__ == "__main__":
    main()
