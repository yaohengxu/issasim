# Isaac Sim 总体架构、数据接入与算法接入

本章把 Isaac Sim 当作一个“可编程的机器人实验系统”来理解：它不是只负责显示 3D 画面，而是将 **场景资产、物理世界、传感器、算法、控制执行和评测** 放入一个可重复运行的闭环。

## 1. 总体架构：六层闭环

```text
                         配置与版本层
        task config / seed / USD / assets / model checkpoint
                                  │
                                  ▼
┌──────────────────────────────────────────────────────────────────┐
│ Isaac Sim / Omniverse Kit Runtime                                  │
│                                                                  │
│  1. 场景资产层      USD Stage / Prim / reference / material      │
│  2. 物理执行层      PhysX / rigid body / articulation / joints   │
│  3. 传感器观测层    RGB / depth / segmentation / lidar / state   │
│  4. 数据适配层      observation、timestamp、坐标、buffer          │
│  5. 算法控制层      FSM / IK / planner / RL / ROS 2 / policy      │
│  6. 评测记录层      reward、success、safety、metrics、artifacts   │
└──────────────────────────────────────────────────────────────────┘
               ▲                                      │
               └──────── action / control command ────┘
```

每一层都有不同职责：

| 层 | 输入 | 输出 | 典型错误 |
| --- | --- | --- | --- |
| 场景资产 | USD、mesh、配置 | Prim tree、材质、层级 | 路径/单位/坐标系混乱 |
| 物理执行 | collider、质量、关节、drive | 下一时刻状态 | 初始穿透、时间步不稳 |
| 传感器 | 物理状态、相机 pose、渲染设置 | 图像/深度/点云/接触 | 时间不同步、外参错误 |
| 数据适配 | 原始 state/sensor | observation | 偷用 GT、shape/单位不一致 |
| 算法控制 | observation、目标 | action | action frame/频率不清楚 |
| 评测记录 | state、action、事件 | success/reward/report | 只看画面、无可回放证据 |

## 2. Isaac Sim 的运行时和时间轴

```text
Python script
   │
   ▼
SimulationApp                 启动 Kit、extension、USD/物理上下文
   │
   ▼
Stage / World                 创建或打开场景，注册对象包装器
   │
   ▼
Timeline play + reset         初始化物理、传感器和 articulation 句柄
   │
   ▼
control loop                  读取 → 算法 → action → step/update → 评测
   │
   ▼
stop + close                  保存结果，释放 GPU / Kit 资源
```

最重要的规则：`SimulationApp(...)` 应先于 `omni.*` 和大部分 `isaacsim.*` API import。Kit 不是普通 Python 库；它需要先建立应用和 extension 上下文。

### 三个频率不要混为一谈

```text
physics:   120 Hz  ── 每个 dt 推进接触和关节动力学
control:    30 Hz  ── 每 4 个 physics step 更新一次 action
sensor:     15 Hz  ── 每 8 个 physics step 获取一次相机观测
render:     15 Hz  ── 需要相机/GUI 时才渲染
```

算法输入必须注明捕获时间，控制 command 也应注明生效时间。对视觉策略尤其要模拟“采集 → 推理 → action”延迟；不能将刚渲染好的未来状态直接提供给控制器。

## 3. 场景接入：如何把世界引入 Isaac Sim

### 3.1 三类场景来源

| 来源 | 常用内容 | 接入方式 | 适用情况 |
| --- | --- | --- | --- |
| 原生/官方 USD | Franka、地面、传感器、材质 | `add_reference_to_stage` | 最快搭建原型 |
| 自己制作的 USD/USDA | 环境、桌面、物体、任务层 | 打开 Stage 或 reference | 可复用场景资产 |
| 外部资产/数据 | CAD、mesh、URDF、日志、标定 | 转换/导入后形成 USD + config | 真实项目资产与数据回灌 |

推荐组织方式：

```text
assets/
  robots/             # 原始或版本化 robot USD
  objects/            # 视觉/碰撞/物理资产
  environments/       # 房间、桌面、工厂、道路
scenes/
  base.usda           # 灯光、世界坐标、PhysicsScene
  task_x.usda         # 对 base/asset 的 reference 与 override
configs/
  task_x.yaml         # seed、初始 pose、控制/评测参数
```

不要把每次实验的随机位置、算法参数写回 robot 原始 USD；USD 管“资产和空间组合”，任务配置管“本次实验意图”。

### 3.2 USD 场景树应该长什么样

```text
/World
├─ /PhysicsScene
├─ /Environment
│  ├─ /Ground
│  ├─ /Table
│  └─ /Lights
├─ /Robots
│  └─ /Franka                 ← reference 到 robot USD
├─ /Objects
│  ├─ /Target
│  └─ /Container
├─ /Sensors
│  └─ /FrontCamera
└─ /Task
   ├─ /GoalMarker
   └─ /Debug
```

路径是代码合同。例如 `/World/Robots/Franka` 一旦作为配置/算法接口使用，就应有加载时校验，而不是在各处硬编码字符串。

### 3.3 Python 场景构建的最小模式

```python
from isaacsim import SimulationApp
app = SimulationApp({"headless": False})

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicSphere

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
target = world.scene.add(DynamicSphere(
    prim_path="/World/Objects/Target",
    name="target",
    radius=0.03,
    position=[0.4, 0.0, 0.2],
    mass=0.05,
))
world.reset()
```

这个片段只创建了一个可被物理推进的物体。生产任务还要明确 collision approximation、摩擦材质、初始无穿透检查、语义标签、版本和 reset 规则。

### 3.4 真实数据怎样进入仿真

真实世界的数据不能直接“塞进仿真器”。需要经过一个可追溯的转换链：

```text
真实日志 / 资产 / 标定
        │
        ├─ 图像/点云：用于感知离线评估、标定、重建或数据质量对齐
        ├─ CAD/mesh/URDF：转换为可组合 USD，并补 visual/collision/physics
        ├─ 位姿/轨迹：转换到统一 map/world frame，成为 reset 或 scenario 参数
        └─ 任务标签：转换为 evaluator 的目标、阈值、事件或 reward
        ▼
USD + ScenarioConfig + RunManifest
        ▼
Isaac Sim scene builder / sensor / evaluator
```

必须保留：来源数据版本、标定版本、坐标变换、单位、处理脚本、随机 seed 和许可证。否则仿真画面可能对，数据链已经不可复现。

## 4. 数据接入与数据输出

### 4.1 四种数据，不要混用

```text
1. Scene data       USD、资产、材质、地图、初始条件
2. Simulator state  关节/刚体 pose、速度、接触、真实 object pose
3. Sensor data      RGB、depth、segmentation、点云、IMU
4. Task data        goal、reward、success、failure reason、metrics
```

仿真 state 是调试和 evaluator 的宝贵真值，但部署到真机的算法通常不能直接读取它。一个合格的 observation 设计应明确哪些字段会给策略，哪些字段只能给评测器。

```text
SUT observation: RGB-D + robot proprioception + route/goal
Evaluator only: 真实物体 pose、接触力、碰撞事件、完整 scene graph
```

### 4.2 统一数据合同

```text
Observation = {
  "timestamp_ns": ...,
  "frame_id": ...,
  "joint_pos_rad": [num_dof],
  "joint_vel_rad_s": [num_dof],
  "rgb": [H, W, 3],
  "depth_m": [H, W],
  "camera_intrinsics": K,
  "T_world_camera": ...,
  "last_action": ...,
}

Action = {
  "timestamp_ns": ...,
  "mode": "joint_position" | "joint_velocity" | "effort" | "cartesian_delta",
  "arm": [...],
  "gripper": [...],
  "frame": "robot_base",
}
```

接口中必须写清 shape、单位、关节顺序、坐标系、时间戳和无效值。数组只有数值没有语义，是机器人系统最常见的集成 bug 来源。

### 4.3 合成数据流水线

```text
scene reset + randomization
       │
       ▼
render products / sensor frames
       │
       ├─ RGB / depth / normal / segmentation
       ├─ 2D/3D boxes / keypoints / pose
       └─ scene metadata / camera pose / randomization params
       ▼
writer → dataset manifest → quality checks → train/validation split
```

数据集不只是一堆 PNG。每个 episode/帧应能反查 scene、资产、语义 schema、相机参数、标注版本和随机化参数。

## 5. 算法如何接入：一个统一适配层

无论使用状态机、IK、传统规划、视觉策略、RL 还是 ROS 2，尽量只让它们面对稳定的环境接口：

```text
               Algorithm / Policy
                        │ observation
                        ▼
                  compute_action()
                        │ abstract action
                        ▼
                Safety / Action Adapter
          clip、limit、frame conversion、IK、delay
                        │ actuator command
                        ▼
             Isaac Sim Articulation / Physics
```

### 5.1 可复用的环境接口

```python
class RobotTaskEnv:
    def reset(self, seed: int, task_config: dict) -> tuple[dict, dict]:
        """设置 scene、随机化、robot 状态和 controller 内部状态。"""

    def observe(self) -> dict:
        """只返回被测算法允许看到的 observation。"""

    def step(self, action: dict) -> tuple[dict, float, bool, bool, dict]:
        """action → safety adapter → physics N steps → observation/reward/termination。"""

    def close(self) -> None:
        """释放 writer、subscription 和 simulation app。"""
```

这与 Gymnasium 风格兼容，但不要求所有任务都做 RL。它的价值是统一 reset、action、结果和日志，便于替换算法或批量评测。

### 5.2 各类算法接入点

| 算法 | 算法输入 | 常见输出 | 接入/转换层 | 要验证什么 |
| --- | --- | --- | --- | --- |
| FSM | task state、阈值、接触 | 下一个状态/夹爪命令 | 状态到 pose/关节目标 | timeout、失败恢复 |
| FK/IK | current q、末端目标、Jacobian | `q_target` 或 `dq` | 限位、速度限制、drive | 奇异、碰撞、跟踪误差 |
| motion planner | start、goal、obstacle、约束 | time-parameterized trajectory | trajectory tracker | 可达、无碰、平滑 |
| visual policy / VLA | RGB/语言/本体状态 | Cartesian delta/trajectory | frame 转换 + IK/safety | 时序、延迟、OOD |
| RL policy | normalized observation | delta-q/torque/skill | action denormalize + safety | reward hacking、泛化 |
| ROS 2 node | topic/TF/action interface | trajectory/joint command | ROS bridge + command arbiter | QoS、clock、TF、超时 |

算法输出不应直接写 USD pose。动态执行应经 articulation joint drive、effort 或经 IK/控制器转换，否则会 teleport 物体/机器人，绕开接触物理。

### 5.3 三种控制层次

```text
任务层：     pick / place / open / recover / stop
                 │ target pose / state transition
运动层：     motion planning / IK / trajectory tracking
                 │ q_target(t) / qdot / tau
执行层：     articulation drive / actuator model / PhysX
                 │
                 ▼
               robot motion
```

初学者应先做“关节空间位置控制”，再做 differential IK，最后才接触抓取、视觉策略和 RL。当前工程的 `demos/04_franka_joint_target_eval.py` 正是关节目标 → articulation → 读回状态 → 成功评测的最小闭环。

## 6. 控制循环与安全层

```text
for each control tick:
    observation = adapter.read_observation()
    raw_action = algorithm.compute_action(observation)
    safe_action = safety.validate_and_clip(raw_action)
    adapter.apply(safe_action)
    simulator.step_physics(control_decimation)
    metrics.update(simulator.state, safe_action)
```

安全层至少负责：关节范围、单周期增量、速度/加速度、workspace、夹爪力、失联超时和 emergency hold。即使是纯仿真训练，也要保留这些约束，防止算法利用穿透、无限驱动或瞬时 teleport 等仿真漏洞。

## 7. 评测系统：如何证明仿真和算法有价值

### 7.1 指标分层

| 层级 | 例子 | 问题 |
| --- | --- | --- |
| 物理稳定 | 穿透、接触、静置、速度爆炸 | 场景本身可信吗？ |
| 感知 | IoU、pose error、延迟、标注有效率 | 输入是否可靠？ |
| 控制 | tracking error、超限率、peak velocity | action 是否可执行？ |
| 任务 | success、drop/collision、time-to-success | 是否完成目标？ |
| 鲁棒性 | 未见 pose/材质/光照/延迟下成功率 | 是否泛化？ |
| 工程 | reset/s、step/s、GPU memory、crash rate | 能否规模化运行？ |

### 7.2 成功判定必须基于状态，不是基于动画

```text
物体放置成功 =
  object 在目标容器水平范围内
  AND object 高度满足容器内部条件
  AND 速度低于稳定阈值
  AND 条件持续 N 个 physics steps
```

对抓取任务也应检查抬升、相对末端 pose、夹爪状态、掉落与碰撞，而不是只看控制器报告 `done`。

### 7.3 三层测试门禁

```text
unit tests
  坐标变换、IK 数学、reward/success predicate
       ▼
headless fixed-seed smoke/regression
  场景加载、N steps、关键 metric、无 crash
       ▼
randomized evaluation + GUI replay
  多 seed、失败分类、视频/传感器 artifacts、统计报告
```

## 8. 一个从零到可展示项目的实施顺序

```text
第 1 步  运行 01_scene_basics.py
        理解 SimulationApp、World、刚体、reset、step。

第 2 步  运行 04_franka_joint_target_eval.py
        修改 HOME/GOAL/阈值，理解 action 和 evaluator。

第 3 步  加入 task config 和 seed
        不再把初始 pose、目标和阈值硬编码。

第 4 步  让 observation 与 evaluator 分离
        策略只读 RGB/本体状态；GT 只给 metrics。

第 5 步  用 IK 或 planner 替换 joint interpolation
        保留相同 action contract 和 safety adapter。

第 6 步  加相机、随机化、日志和失败回放
        输出 manifest、成功率、失败原因和短视频。

第 7 步  将任务封装为 reset/step 环境
        可接 RL、ROS 2、VLA 或批量 evaluator。
```

## 9. 面试时的总体表述

> 我把 Isaac Sim 作为可编程的闭环实验系统，而不仅是可视化工具。USD 负责组合和版本化机器人、环境与传感器资产；PhysX 和 articulation 负责物理/执行；传感器层产生与真实部署一致的 observation；算法通过统一 action adapter 接入，并经过限位、坐标转换、IK 或 drive 后执行；评测层使用真值状态、接触和任务条件定义成功与安全指标。这样场景、数据、算法和评测可分别替换，也可以通过 seed、配置、资产和模型版本形成可回放的实验记录。

## 10. 相关文档

- `08_IsaacSim架构与生命周期.md`：Kit、Stage、Timeline 与脚本生命周期。
- `09_USD场景描述与资产工程.md`：USD 分层、reference、variant 和资产规范。
- `11_传感器观测与合成数据.md`：相机、标注、随机化与数据质量。
- `12_控制代码接入与系统集成.md`：ROS 2、OmniGraph、控制器 adapter。
- `13_任务工程化与验证.md`：测试、回归、失败分类与性能工程。
- `16_从零编写IsaacSim机器人程序.md`：可直接跟做的 Franka 编程教程。
