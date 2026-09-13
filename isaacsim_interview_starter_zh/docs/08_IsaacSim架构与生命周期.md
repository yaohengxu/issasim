# Isaac Sim 架构与脚本生命周期

## 1. 先分清四个名字

```text
NVIDIA Omniverse Kit
  └─ 可扩展实时应用运行时：窗口、USD、extension、事件循环、渲染
       └─ Isaac Sim
            ├─ 机器人、物理、传感器、Replicator、ROS 等扩展集合
            ├─ USD Stage（当前场景）
            └─ PhysX / RTX 等后端
                 └─ 你的 Python、Extension、OmniGraph 或 ROS 节点
```

- **Kit** 是宿主应用；Isaac Sim 不是普通 Python 库，Python 只是驱动 Kit 的一种入口。
- **Extension** 是可启停的功能包。缺少扩展时，模块 import 成功也可能没有运行时功能。
- **USD Stage** 是当前打开的场景图；它保存描述，不等于物理求解器的运行状态。
- **PhysX** 根据 Stage 上的 physics schema 建立并推进物理对象；它不是“每个可见 Prim 自动有物理”。

面试陷阱：不要把 USD、PhysX、Kit 和 Isaac Lab 混成一个东西。USD 管描述，PhysX 管动力学，Kit 管应用与扩展，Isaac Lab 通常承接学习任务与训练流程。

## 2. 两个运行形态

| 形态 | 适用 | 入口 | 优点 | 注意 |
| --- | --- | --- | --- | --- |
| Standalone Python | 批处理、原型、回归测试、训练 worker | `python.bat script.py` | 易版本管理、可 headless | 必须自己管理 App 生命周期 |
| Extension / GUI | 交互工具、调参面板、菜单工作流 | Kit 启动并启用 extension | 与编辑器深度整合 | 需处理加载、卸载和事件订阅 |

本工程的 demo 使用 Standalone，因为它最接近 CI/训练中的调用方式。GUI 中做出的场景应保存为 USD，再用 standalone 做可复现执行；不要把关键任务逻辑只留在手工点击里。

## 3. Standalone 的正确生命周期

```text
解析自己的 CLI 参数
        │                         注意：把剩余 Kit 参数留给 Kit
        ▼
SimulationApp({headless: ...})  ← 必须在 omni.* / isaacsim.* API import 前
        ▼
import USD / Isaac / PhysX API
        ▼
创建或打开 Stage → 添加资产 → 配置物理、传感器、控制器
        ▼
play / reset / update             ← 等待资产和物理句柄初始化
        ▼
while app.is_running():
    read observation
    policy/controller(obs) -> action
    apply action
    step/update physics (+ render if needed)
    log + check termination
        ▼
保存必要结果 → stop timeline → simulation_app.close()
```

`SimulationApp` 的初始化顺序是高频 bug：Kit 扩展和上下文尚不存在时就 import `omni.*`，会出现模块不可用、上下文为空或非确定性初始化。`demos/01_scene_basics.py` 和 `02_franka_joint_control.py` 都把 App 创建放在 Isaac API import 之前。

## 4. Stage、Timeline、World 的职责

```text
Stage:  Prim 树、属性、reference、layer、材质、物理 schema
  │
  ├─ Timeline: play / pause / stop，控制仿真时间流
  │
  └─ World: 高层便利包装，常用于 scene registry、reset、step
       └─ Articulation / RigidPrim 等 wrapper：读写特定 Prim 的运行状态
```

- Stage 编辑可发生在暂停状态；物理状态通常只在 play/step 后变化。
- `reset` 重新初始化物理与 wrapper 状态，适合 episode 起点；不是只把画面复原。
- `update()` 推进 Kit 帧，`World.step()` 是带场景管理的高层路径。项目内选定一种主循环，避免两个循环竞争推进。

## 5. 本机 6.0.1 的 API 选择

安装包和历史教程同时存在不同层的接口，例如 `isaacsim.core.api.World`、`isaacsim.core.api.objects`，以及 `isaacsim.core.experimental.*`。两者都可能出现在示例中；项目最重要的是**同一个功能模块内不混用对象生命周期和张量后端**。

建议：

1. 以当前安装版本的内置示例和 API 文档为准，不直接复制旧版 `omni.isaac.*` 教程。
2. 一个 demo 固定一种包装层；明确其 reset、坐标和数组类型约定。
3. 在 `requirements` 或 README 记录 Isaac Sim 版本、启用的 extension 和资产版本。

## 6. 时间、频率与吞吐

```text
物理步 dt_phys = 1/120 s ─┐
                            ├─ 4 次物理 step → 1 次控制 action（30 Hz）
渲染 dt_render = 1/30 s ───┘
```

控制、物理、相机不必同频。低层接触可能要更高物理频率；视觉策略可能每几步取一帧；训练时可关渲染以提升吞吐。日志要同时记录 `sim_time`、物理 step、控制 step，不能只用墙钟时间判断性能。

## 7. 面试可用的初始化骨架

```python
from isaacsim import SimulationApp
app = SimulationApp({"headless": False})

# 只能在 app 之后 import
from isaacsim.core.api import World

world = World(stage_units_in_meters=1.0)
# add/load USD, configure sensors, build controller
world.reset()
while app.is_running():
    observation = read_observation()
    action = controller.compute(observation)
    apply_action(action)
    world.step(render=True)
app.close()
```

这段代码本身不是完整任务；面试时还要补上 action 的控制频率、异常处理、成功/失败判定、seed 和日志。
