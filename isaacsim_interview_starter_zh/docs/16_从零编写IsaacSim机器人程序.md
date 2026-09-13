# 从零编写 Isaac Sim 机器人程序

目标：读懂并改写 `demos/01_scene_basics.py`、`02_franka_joint_control.py`、`04_franka_joint_target_eval.py`，最终能自己实现一个“机器人从初始位姿运动到目标并评测结果”的任务。

## 0. 先记住程序的总骨架

```text
CLI 配置
   │
   ▼
启动 SimulationApp
   │
   ▼
创建/打开 USD Stage ──> 添加环境、机器人、物体、传感器
   │
   ▼
reset / play / update ──> 等待物理与 articulation 初始化
   │
   ▼
控制循环：观测 → action → 安全检查 → 下发 → physics step → 评测/记录
   │
   ▼
停止 timeline、关闭 App
```

任何 Isaac Sim 程序都可以放进这个框架。初学时不要同时写复杂场景、视觉、IK、RL 和 ROS；每次只增加一个闭环环节。

## 1. 如何运行脚本

必须使用 Isaac Sim 的 Python 环境：

```powershell
cd D:\sofaware2\issasim\isaac-sim-standalone-6.0.1-windows-x86_64
.\python.bat ..\isaacsim_interview_starter_zh\demos\01_scene_basics.py
.\python.bat ..\isaacsim_interview_starter_zh\demos\04_franka_joint_target_eval.py --seconds 8
```

无界面测试会更快、更适合反复调代码：

```powershell
.\python.bat ..\isaacsim_interview_starter_zh\demos\04_franka_joint_target_eval.py --headless --seconds 8
```

不要用系统 `python.exe`。它没有 Kit 的 extension 路径和 Isaac Sim 所依赖的运行时库。

## 2. 第一行关键接口：`SimulationApp`

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": args.headless})

# 现在才可以 import omni.* 或 isaacsim.core.*
```

这一步启动 Kit 应用，加载核心 extension，创建 USD/渲染/物理上下文。因而导入顺序不是风格问题，而是运行条件。

```text
错误顺序： import omni.usd → SimulationApp()  # 上下文可能尚未存在
正确顺序： SimulationApp() → import omni.usd  # Kit 已准备好
```

脚本末尾使用 `try/finally` 调用 `simulation_app.close()`，防止出错后遗留 Kit/GPU 进程。

## 3. 场景有两种来源：创建或打开 USD

### A. 用 Python 创建最小场景

`01_scene_basics.py` 使用 `World` 和高层对象 wrapper：

```python
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
ball = world.scene.add(DynamicSphere(
    prim_path="/World/Ball", name="ball", radius=0.05,
    position=np.array([-0.35, 0.0, 0.75]), mass=0.05,
))
world.reset()
```

你在这里写的是“场景构建代码”。`prim_path` 会成为 USD Stage 内的地址；`name` 是 `World.scene` 的 Python 访问名。动态球具有刚体和碰撞语义，地面提供静态碰撞。

### B. 引入一个已有的 USD 资产

机器人通常不手写几百个 link，而是 reference 一个 USD：

```python
stage_utils.create_new_stage(template="default stage")
assets_root = get_assets_root_path()
stage_utils.add_reference_to_stage(
    usd_path=assets_root + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
    path="/World/Franka",
    variants=[("Gripper", "AlternateFinger")],
)
```

```text
当前 Stage
/World/Franka  ── reference ──> franka.usd
                                      ├─ links
                                      ├─ joints / drives
                                      ├─ collision geometry
                                      └─ visual meshes
```

`path` 是把资产挂到当前场景的地址，不是资产磁盘路径。reference 后的对象可以通过 wrapper 控制；不要直接逐个修改机器人 link 的 pose。

### C. 使用文本 USD 场景

[`assets/tabletop_task_template.usda`](../assets/tabletop_task_template.usda) 展示了文本场景文件。实际工程通常将环境、机器人、物体分成 USD 层，再用一个任务脚本在 reset 时改初始状态。

## 4. 场景物理设置该怎么写

```text
静态环境：visual + collider
可抓物体：visual + collider + rigid body + mass + material
机器人：articulation root + links + joints + drives + colliders
```

`FixedCuboid`（桌子/障碍）和 `DynamicSphere`（球）是学习期最安全的对象。创建物体后必须 `world.reset()`，再进循环。改变物体初始 pose 可以在 reset 做；执行过程中要让 PhysX 和机器人 drive 产生运动，而不是每帧 set pose。

## 5. 如何获取“可控的机器人”

```python
robot = Articulation("/World/Franka")
robot.set_default_state(dof_positions=HOME.tolist())

timeline = omni.timeline.get_timeline_interface()
timeline.play()
simulation_app.update()
robot.reset_to_default_state()
```

`Articulation` 是机器人多刚体系统的控制/状态包装。`update()` 之后运行时对象才完成初始化；默认关节状态应在每个 episode reset 后恢复。

先打印并确认 DOF 的名字和数量，再使用 index。Franka 例子是 7 个 arm DOF 加 2 个 finger DOF，但这不是所有机器人通用规律。

## 6. 最简单的控制：关节位置目标

`04_franka_joint_target_eval.py` 的控制接入点只有一行：

```python
action = make_action(elapsed)                 # 你的 controller 的输出
robot.set_dof_position_targets(action.tolist())  # 发送到 articulation drive
simulation_app.update()                       # 推进一帧
```

```text
make_action() → q_target[9]
       │
       ▼
set_dof_position_targets()
       │              USD joint position drive（常近似为 PD）
       ▼
PhysX 推进机器人 ──> get_dof_positions() 读回 q_actual
```

`make_action()` 目前是 HOME 到 GOAL 的 smoothstep 插值：

```python
phase = np.clip(elapsed / 1.0, 0.0, 1.0)
smooth_phase = phase * phase * (3.0 - 2.0 * phase)
action = HOME + smooth_phase * (GOAL - HOME)
```

它避免目标瞬间跳变。你可以先尝试把 `GOAL[0]` 改小 0.1 rad、把时长改为 3 秒，然后观察终端中的 target 与 actual 差异。

## 7. 如何写仿真评测，而不是只看画面

```python
current = robot.get_dof_positions().numpy()[0]
arm_error = np.linalg.norm(current[:7] - GOAL[:7])
success = arm_error <= 0.08
```

这是关节空间任务的评测。它的契约是：误差只看 7 个 arm DOF，单位 rad，使用 L2 norm，阈值 0.08。真实抓取/放置任务要替换为物体相对容器、末端、速度、接触和持续时间的综合条件。

```text
控制器完成（controller done） ≠ 物理任务成功
关节误差小                    ≠ 末端没有碰撞
画面看着接近                  ≠ 指标通过
```

每次运行至少记录：seed、dt、控制频率、GOAL、final error、成功与否、失败原因。这样改参数后才能比较。

## 8. 你将如何引入不同控制算法

不要改动场景和评测接口，只替换 `make_action` 的实现：

| 算法 | 输入 | `make_action` / adapter 输出 | 下层接口 |
| --- | --- | --- | --- |
| 轨迹插值 | time、起终点 | `q_target` | position targets |
| PID/关节伺服 | `q, q_dot, q_goal` | position/velocity/effort | 对应 drive/effort |
| differential IK | 末端位置误差、Jacobian | `q_current + dq` | position targets |
| motion planner | start、goal、obstacle | 时间参数化关节轨迹 | position targets |
| RL policy | observation | delta-q / Cartesian delta | safety + IK/drive |
| 行为树/FSM | 任务状态、传感器 | 高层目标/夹爪命令 | planner + gripper |

统一原则：**算法输出必须注明 frame、单位、关节顺序和频率**，再经 safety adapter 下发。`franka_wave_demo.py` 是 differential IK 的下一步；`franka_ball_to_bowl.py` 是抓放状态机的下一步。

## 9. 从关节目标升级到 IK

```text
目标末端位置 x_goal
       │
当前关节 q ── FK ──> 当前末端 x
       │                    │
       └─ Jacobian J <──── error dx = x_goal - x
                            │
                    dq = Jᵀ(JJᵀ + λ²I)⁻¹ dx
                            │
                    q_target = clip(q + αdq)
                            │
                    set_dof_position_targets(q_target)
```

先运行 `demos/03_differential_ik_math.py` 看纯数学输出；再读现有 `user_examples/franka_wave_demo.py`。新增 IK 时要额外处理 joint limit、速度限制、奇异点阻尼、目标 frame、末端姿态和碰撞；只用位置误差通常只能做演示级 pre-grasp。

## 10. 从 IK 升级到抓取任务

```text
reset
  → open gripper
  → move to pre-grasp
  → slow approach
  → close gripper
  → verify grasp
  → lift
  → move to pre-place
  → release
  → verify object stable in goal region
```

这是状态机，而不是一个 `if`。每个状态需要 timeout 和失败恢复。现有 `franka_ball_to_bowl.py` 中的 fixed joint 是任务级抓取抽象；若追求真实接触抓取，要删除该便利约束，调夹爪接触、摩擦、质量和控制参数，并统计掉落率。

## 11. 推荐练习路径（照做即可）

1. 跑 `01_scene_basics.py`，改球高度/质量/摩擦，解释差异。
2. 跑 `02_franka_joint_control.py`，只改变一个关节摆动幅度。
3. 跑 `04_franka_joint_target_eval.py`，修改 `GOAL`、阈值、插值时长，观察 success。
4. 为 `04` 添加 `--goal-scale` 参数，并写入终端日志。
5. 将 `make_action` 改为固定 `GOAL`，比较瞬间跳变与平滑轨迹。
6. 读 `03_differential_ik_math.py` 和 `franka_wave_demo.py`，解释每个矩阵维度。
7. 读 `franka_ball_to_bowl.py`，画状态机，并给失败加入分类日志。

完成前四步，你已经能独立写出最小 Isaac Sim 机械臂关节控制程序；完成后四步，就能向一个可评测的具身抓取项目扩展。
