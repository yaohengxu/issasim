# Isaac Sim 6.0.1 核心 API 速查与编码规范

这不是替代官方 API 文档的完整索引，而是本工程和本机 6.0.1 内置示例中最常用的工作流速查。版本迭代时，先以本机 `exts/` 下示例和当前 API 文档为准。

## 1. 常用对象与职责

| 概念/API | 作用 | 典型时机 |
| --- | --- | --- |
| `SimulationApp` | 启动/关闭 Kit 应用 | 脚本最前、脚本最后 |
| `World` | 高层场景注册、reset、`step` | 常规 standalone demo |
| `stage_utils.create_new_stage` | 创建 USD Stage | 实验场景初始化 |
| `stage_utils.add_reference_to_stage` | 将 USD 资产引用到 Prim path | 加载机器人/环境资产 |
| `Articulation` | 多关节机器人状态、Jacobian、DOF target | 关节控制 / IK |
| `DynamicSphere` / `FixedCuboid` | 高层刚体/静态对象包装 | 最小物理场景 |
| `PhysicsMaterial` | 摩擦、恢复系数 | 接触物体配置 |
| `omni.timeline` | play/stop 时间线 | experimental API 主循环 |
| `get_assets_root_path` | 取得官方资产根路径 | 引用随安装提供的资产 |

## 2. 最小脚本骨架 A：`World` 场景管理

```python
import sys
from isaacsim import SimulationApp

app = SimulationApp({"headless": False})
# App 之后再 import Isaac/Omniverse API
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicSphere

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
ball = world.scene.add(DynamicSphere(
    prim_path="/World/Ball", name="ball", radius=0.05,
    position=[0, 0, 0.5], mass=0.05,
))
world.reset()
while app.is_running():
    world.step(render=True)
    position, orientation = ball.get_world_pose()
app.close()
```

对应本工程 `demos/01_scene_basics.py`。`scene.add` 的意义是让 wrapper 由 `World` 生命周期管理并可按 name 查询；`prim_path` 是 USD 场景中稳定的路径。

## 3. 最小脚本骨架 B：引用资产与 articulation

```python
from isaacsim import SimulationApp
app = SimulationApp({"headless": False})

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.timeline
from isaacsim.core.experimental.prims import Articulation
from isaacsim.storage.native import get_assets_root_path

stage_utils.create_new_stage(template="default stage")
root = get_assets_root_path()
stage_utils.add_reference_to_stage(
    usd_path=root + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
    path="/World/Franka",
    variants=[("Gripper", "AlternateFinger")],
)
robot = Articulation("/World/Franka")
timeline = omni.timeline.get_timeline_interface()
timeline.play()
app.update()                     # 允许运行时 wrapper 初始化
robot.set_dof_position_targets([0.0] * 9)
app.update()
timeline.stop()
app.close()
```

对应 `demos/02_franka_joint_control.py` 与已有 `franka_wave_demo.py`。DOF 数、关节顺序、关节名称不能猜：应从加载的机器人 articulation 查询/打印，或固化在机器人配置中校验。

## 4. 读状态、写动作的约定

```text
读：articulation / rigid prim / sensor → 归一化为 observation
算：controller(obs) or policy(obs)    → 明确定义 action type/frame/unit
查：clip + limit + collision/safety    → safe_action
写：set_dof_position_targets / effort  → 机器人 drive
推：step/update N 次                  → 下一帧状态
```

建议集中写一个 `RobotAdapter`，将具体 API 封装为以下方法：

```python
robot.get_joint_positions()       # 返回固定 joint order 的 rad
robot.get_end_effector_pose()     # 返回明确 frame 的 m + quaternion
robot.apply_position_target(q)    # 输入已限幅的 rad
robot.open_gripper()
robot.close_gripper()
```

不要让上层 RL/规划代码直接调用各种 `set_*` API；这会把 Prim path、张量类型和版本差异散落到各处。

## 5. 数组与坐标的编码规范

| 项 | 推荐约定 |
| --- | --- |
| 距离 | 米（m） |
| 角度 | 弧度（rad） |
| 关节顺序 | 按 DOF name 明确记录，不能只靠 index 注释 |
| 四元数 | 在接口注释写清顺序，如 `wxyz`；入参前 normalize/校验 |
| 变换 | 函数名显式写方向，例如 `T_world_camera` |
| 时间 | `sim_time`、physics step、control step 同时记录 |
| batch | 明确 shape，例如 `[num_env, num_dof]`，避免单环境代码隐式广播 |

## 6. Extension、依赖与启动参数

有些功能来自 extension，而不是 import 本身。例如操纵器 controller、ROS bridge、Replicator、某种传感器都可能需要被启用。工程应在以下任一位置明确声明依赖：Kit `.kit` 配置、extension 的 `extension.toml`、或 `SimulationApp` 的启动参数；并在启动时记录实际启用的版本。

```text
功能不工作
├─ Python 模块未找到              → 检查 extension 是否启用/版本匹配
├─ Prim 找不到                    → 检查 USD 是否加载完、path 是否正确
├─ wrapper state 无效             → 检查 timeline / reset / update 顺序
└─ headless 与 GUI 表现不同       → 检查 renderer、传感器和 render 调用
```

## 7. 资源释放和异常安全

Standalone 中务必确保关闭 App：

```python
try:
    main()
finally:
    simulation_app.close()
```

这样在资产缺失、控制器异常或测试失败时也能释放 Kit 进程和 GPU 资源。批量 evaluation 中还应为单个 episode 的异常记录 seed/配置，而不是吞掉错误后继续产出不可信指标。

## 8. API 迁移时的最小检查

1. 确认安装版本和 release notes，而不是只看网页旧教程。
2. 从当前版本 `exts/` 内一个最接近的官方示例开始。
3. 先运行 1 个 headless smoke episode，再跑 GUI。
4. 校验 Prim path、DOF 名称/顺序、数组 device/shape 和 reset 行为。
5. 锁定版本并在项目 README 记录已验证命令。
