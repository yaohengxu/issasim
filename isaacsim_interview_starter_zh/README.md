# Isaac Sim 具身仿真面试速成包（中文）

面向 Isaac Sim 初学者的可运行小工程。目标不是背 API，而是能在面试中把一条具身仿真链路讲清楚：**场景与资产 → 物理 → 观测 → 控制 → 任务评估 → Sim-to-Real**。

本工程适配本机的 Isaac Sim 6.0.1；已有的 `user_examples` 保持不改动。

## 目录

```text
isaacsim_interview_starter_zh/
├─ README.md
├─ docs/
│  ├─ README.md                         # 面试知识地图与阅读顺序
│  ├─ 00_学习路线.md
│  ├─ 01_核心概念.md
│  ├─ 02_场景物理与USD.md
│  ├─ 03_机器人控制与IK.md
│  ├─ 04_具身任务与训练.md
│  ├─ 05_高频面试题.md
│  ├─ 06_排错清单.md
│  ├─ 07_周末冲刺计划.md
│  ├─ 08_IsaacSim架构与生命周期.md
│  ├─ 09_USD场景描述与资产工程.md
│  ├─ 10_物理仿真与数值稳定性.md
│  ├─ 11_传感器观测与合成数据.md
│  ├─ 12_控制代码接入与系统集成.md
│  ├─ 13_任务工程化与验证.md
│  ├─ 14_岗位面试工程审查清单.md
│  ├─ 15_核心API速查与编码规范.md
│  ├─ 16_从零编写IsaacSim机器人程序.md
│  ├─ 17_IsaacSim总体架构_数据与算法接入.md
│  └─ 18_Sim2Real与数字孪生.md
├─ assets/
│  └─ tabletop_task_template.usda       # 可阅读的 USD 场景模板
└─ demos/
   ├─ 01_scene_basics.py
   ├─ 02_franka_joint_control.py
   ├─ 03_differential_ik_math.py
   └─ 04_franka_joint_target_eval.py
```

## 先运行什么

从 Isaac Sim 安装根目录运行（不是从本工程目录直接使用系统 Python）：

```powershell
cd D:\sofaware2\issasim\isaac-sim-standalone-6.0.1-windows-x86_64
.\python.bat ..\isaacsim_interview_starter_zh\demos\01_scene_basics.py
.\python.bat ..\isaacsim_interview_starter_zh\demos\02_franka_joint_control.py --seconds 15
.\python.bat ..\isaacsim_interview_starter_zh\demos\03_differential_ik_math.py
.\python.bat ..\isaacsim_interview_starter_zh\demos\04_franka_joint_target_eval.py --seconds 8
```

无界面冒烟测试：

```powershell
.\python.bat ..\isaacsim_interview_starter_zh\demos\01_scene_basics.py --headless --max-steps 240
.\python.bat ..\isaacsim_interview_starter_zh\demos\02_franka_joint_control.py --headless --seconds 5
.\python.bat ..\isaacsim_interview_starter_zh\demos\04_franka_joint_target_eval.py --headless --seconds 8
```

> 首次加载 Franka 时需要能访问 Isaac Sim 资产库；若报找不到 assets，请在安装目录执行 `post_install.bat`，再重试。

## 推荐阅读顺序

1. 运行 `01_scene_basics.py`，理解仿真循环和刚体。
2. 从 `docs/README.md` 按架构图阅读；重点先读 `08`、`09`、`10`。
3. 运行 `02_franka_joint_control.py` 与 `04_franka_joint_target_eval.py`，并按 `16_从零编写IsaacSim机器人程序.md` 逐步修改代码。
4. 阅读 `11_传感器观测与合成数据.md`、`13_任务工程化与验证.md`、`14_岗位面试工程审查清单.md`，用自己的项目经历替换答案中的占位表达。
5. 面试前按 `05_高频面试题.md` 和 `07_周末冲刺计划.md` 演练。

## 与现有 demo 的关系

本工程的 demo 只覆盖最小可解释闭环；现有 `user_examples` 适合进阶演示：

| 现有文件 | 可以在面试中说明的能力 |
| --- | --- |
| `franka_wave_demo.py` | 动态刚体、Jacobian、阻尼最小二乘差分 IK、末端跟踪误差 |
| `franka_ball_to_bowl.py` | PickPlaceController、夹爪、PhysX 碰撞、抓取状态机、任务成功判定 |
| `franka_soft_cloth_newton_demo.py` | 可变形物体/布料与 Newton 物理路径 |

运行它们的方式相同，例如：

```powershell
.\python.bat .\user_examples\franka_ball_to_bowl.py --max-steps 900
```

## 面试时的一句话框架

“我用 USD 组织机器人和场景资产，用 PhysX 推进接触动力学；控制器根据观测产生关节动作，通过 articulation 的驱动器执行；任务层把抓取、抬升、放置拆成状态，并以物体位姿和接触等指标评估。为减小 Sim-to-Real 差距，会随机化摩擦、质量、相机、光照和控制延迟。”
