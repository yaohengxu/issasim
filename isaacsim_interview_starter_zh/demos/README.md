# Isaac Sim Demo 运行与学习指南

本目录按难度收集了 7 个可运行示例。前 4 个是学习工程编写的最小闭环；后 3 个由原 `user_examples` 纳入，用于学习差分 IK、抓放状态机和可变形布料。

```text
01 场景与刚体
        ↓
02 关节目标控制
        ↓
03 DLS IK 数学
        ↓
04 关节目标 + 评测闭环
        ↓
05 Franka 差分 IK 跟踪
        ↓
06 Franka 物理抓放
        ↓
07 Franka + Newton 布料
```

## 0. 统一运行方法

必须从 Isaac Sim 安装根目录，使用它自带的 `python.bat`。不要使用系统 Python，也不要在 `demos` 目录直接输入 `python xxx.py`。

```powershell
cd D:\sofaware2\issasim\isaac-sim-standalone-6.0.1-windows-x86_64
```

首次启动会加载 Kit extension，通常需要几十秒。首次引用 Franka 或公开示例资产时还可能需要网络和缓存；若资产根找不到，先执行：

```powershell
.\post_install.bat
```

GUI 演示不要加入 `--headless`。`--headless` 只用于终端冒烟测试和批量回归。

## 1. Demo 清单

| 文件 | GUI 中观察什么 | 核心学习点 | 推荐命令 |
| --- | --- | --- | --- |
| `01_scene_basics.py` | 红球受重力下落并碰撞蓝色障碍/地面 | `SimulationApp`、`World`、刚体、材质、`step` | `...\01_scene_basics.py` |
| `02_franka_joint_control.py` | Franka 一个关节平滑摆动 | USD reference、Articulation、关节位置 target | `...\02_franka_joint_control.py --seconds 15` |
| `03_differential_ik_math.py` | 无 GUI；终端输出 `dx → dq` | Jacobian、阻尼最小二乘 DLS | `...\03_differential_ik_math.py` |
| `04_franka_joint_target_eval.py` | Franka 从 HOME 平滑到 GOAL | action、状态读取、关节误差、success evaluator | `...\04_franka_joint_target_eval.py --seconds 8` |
| `05_franka_wave_demo.py` | 红色目标落下；Franka 末端持续向其预抓取点跟踪 | 动态刚体、Jacobian、差分 IK、tracking metric | `...\05_franka_wave_demo.py --seconds 20` |
| `06_franka_ball_to_bowl.py` | Franka 抓球、搬运、释放到蓝色碗中 | PickPlaceController、夹爪、PhysX、状态机、任务判定 | `...\06_franka_ball_to_bowl.py --max-steps 900` |
| `07_franka_soft_cloth_newton_demo.py` | Franka 与可变形布料交互 | Newton、布料、VBD、机器人-软体接触 | `...\07_franka_soft_cloth_newton_demo.py` |

表中的 `...` 统一替换为：

```text
..\isaacsim_interview_starter_zh\demos
```

## 2. 可直接复制的 GUI 命令

```powershell
# 01：基础刚体；窗口关闭前持续运行
.\python.bat ..\isaacsim_interview_starter_zh\demos\01_scene_basics.py

# 02：关节位置控制；15 秒自动退出
.\python.bat ..\isaacsim_interview_starter_zh\demos\02_franka_joint_control.py --seconds 15

# 03：无 GUI，先看 DLS IK 数学
.\python.bat ..\isaacsim_interview_starter_zh\demos\03_differential_ik_math.py

# 04：完整的最小控制 + 指标闭环；8 秒自动退出
.\python.bat ..\isaacsim_interview_starter_zh\demos\04_franka_joint_target_eval.py --seconds 8

# 05：可视化差分 IK；红球目标；20 秒自动退出
.\python.bat ..\isaacsim_interview_starter_zh\demos\05_franka_wave_demo.py --seconds 20

# 将红球改成红色立方体
.\python.bat ..\isaacsim_interview_starter_zh\demos\05_franka_wave_demo.py --target-shape cube --seconds 20

# 06：物理抓放；最多 900 个仿真 step 后退出
.\python.bat ..\isaacsim_interview_starter_zh\demos\06_franka_ball_to_bowl.py --max-steps 900

# 07：Newton 布料交互；会弹出 Newton 自己的 GLFW Viewer 窗口
.\python.bat ..\isaacsim_interview_starter_zh\demos\07_franka_soft_cloth_newton_demo.py
```

## 3. 每个 demo 应该观察与解释什么

### 01_scene_basics.py：可见、可碰撞、受动力学影响是三回事

观察红球是否受重力、地面和障碍物碰撞影响。打开代码，尝试只改一个变量：球的初始高度、质量、摩擦或恢复系数。面试时说明：visual、collider、rigid body、physics material 分别承担不同职责。

### 02 与 04：机器人关节控制最小闭环

```text
HOME/GOAL → q_target
             │
             ▼
set_dof_position_targets(q_target)
             │
             ▼
USD joint drive / PhysX
             │
             ▼
get_dof_positions() → error → success
```

先运行 02 看单关节，再运行 04 看完整 `action → state → evaluator`。练习修改 `04_franka_joint_target_eval.py` 的 `GOAL`、`success-threshold` 和 `make_action()` 的插值时长；每次只改一项。

### 03 与 05_franka_wave_demo.py：从数学到真实机械臂 IK

`03_differential_ik_math.py` 只演示：

```text
dq = Jᵀ (J Jᵀ + λ² I)^-1 dx
```

`05_franka_wave_demo.py` 则从 Franka articulation 读取真实 Jacobian 和末端 pose，计算 `dq` 后下发 7 个关节目标。GUI 中注意：它跟踪的是目标物体上方的预抓取点，不是故意把手伸进球心。终端每秒打印 tracking error，最后输出 success 阈值判断。

### 06_franka_ball_to_bowl.py：任务状态机和抓放评测

```text
open → approach → close → attach/verify → lift → move → release → evaluate
```

球、碗壁和地面都具有 PhysX 碰撞。为了让教学任务稳定，脚本在满足接近条件后创建 fixed joint 来抽象“抓住”；这不是纯摩擦接触抓取。终端会打印 `Grasp attached`、`Ball released`、`Task finished` 和 `in_bowl`，这些是比“画面看上去成功”更可靠的任务证据。

### 07_franka_soft_cloth_newton_demo.py：可变形物体

它转发到 Isaac Sim 自带的 Newton cloth Franka 示例，重点是布料不是刚体：它有网格形变、软体求解和接触稳定性问题。

`warp` 不是 Newton 普通 Python 包的一部分，而是由 Isaac Sim 的 `omni.warp.core` extension 提供。因此 wrapper 会先启动一个无 Isaac 主窗口的 `SimulationApp` 来加载 Warp 和 Newton，再由 Newton 弹出自己的 GLFW Viewer。不要直接运行安装目录中的 `example_cloth_franka.py`，否则会出现 `ModuleNotFoundError: No module named 'warp'`。

首次启动后等待终端出现 `app ready`，随后会出现布料窗口；若官方示例资产尚未缓存，首次还可能需要下载。可先做两帧无窗口验收：

```powershell
.\python.bat ..\isaacsim_interview_starter_zh\demos\07_franka_soft_cloth_newton_demo.py --viewer gl --headless --num-frames 2 --quiet
```

Newton 的 OpenGL GUI 还需要 `pyglet`。若终端提示 `Newton GUI requires pyglet`，从 Isaac Sim 安装根目录执行一次：

```powershell
.\python.bat -m pip install pyglet
```

这只补充 Newton Viewer 的窗口依赖；不要为 07 单独安装 `warp`。

07 在首次运行时还会从 Newton 的公开资产库下载 Franka 模型，因此需要 `GitPython` 和可用的 `git` 命令。若提示 `GitPython package is required`，执行一次：

```powershell
.\python.bat -m pip install GitPython
```

下载后的资产会缓存在 `%LOCALAPPDATA%\newton-physics`；后续运行通常不再下载。

查看 Newton 可用参数：

```powershell
.\python.bat ..\isaacsim_interview_starter_zh\demos\07_franka_soft_cloth_newton_demo.py --help
```

## 4. 无界面测试命令

```powershell
.\python.bat ..\isaacsim_interview_starter_zh\demos\01_scene_basics.py --headless --max-steps 240
.\python.bat ..\isaacsim_interview_starter_zh\demos\02_franka_joint_control.py --headless --seconds 5
.\python.bat ..\isaacsim_interview_starter_zh\demos\04_franka_joint_target_eval.py --headless --seconds 8
.\python.bat ..\isaacsim_interview_starter_zh\demos\05_franka_wave_demo.py --headless --seconds 10
.\python.bat ..\isaacsim_interview_starter_zh\demos\06_franka_ball_to_bowl.py --headless --max-steps 900
.\python.bat ..\isaacsim_interview_starter_zh\demos\07_franka_soft_cloth_newton_demo.py --viewer gl --headless --num-frames 2 --quiet
```

有界面验证“看起来是否正确”；无界面验证“能否稳定、批量、可复现地运行”。正式工程两者都需要。

## 5. 常见问题

| 现象 | 先检查什么 |
| --- | --- |
| `No module named isaacsim` | 是否从安装根目录用 `python.bat` 启动？ |
| `No module named warp`（07） | 确认使用学习工程中更新后的 `07` wrapper；不要直接执行 Newton 安装目录内的源文件，也不需要自行 `pip install warp` |
| `Newton GUI requires pyglet`（07） | 从 Isaac Sim 安装根目录运行 `.\python.bat -m pip install pyglet`，然后重新执行 07 |
| `GitPython package is required`（07） | 从 Isaac Sim 安装根目录运行 `.\python.bat -m pip install GitPython`；确保终端中的 `git --version` 可用 |
| 找不到 assets root | 运行 `post_install.bat`，检查网络与资产缓存 |
| GUI 启动慢 | Kit 首次加载 extension/Shader cache 正常，等待终端出现 `app ready` |
| Franka 不动 | 是否 `reset/play/update` 完成；Prim path、DOF 数量、target 维度是否匹配？ |
| 抓放没成功 | 先看 `Grasp attached` 与 `in_bowl` 日志；不要只看录屏 |
| headless 结果不同 | 检查 render/sensor、资产加载、固定 step/seconds 和随机 seed |

## 6. 来源与修改说明

- `05_franka_wave_demo.py` 与 `06_franka_ball_to_bowl.py` 纳入自本机原 `user_examples`，逻辑保持一致，运行命令改为新的 `demos` 路径。
- `07_franka_soft_cloth_newton_demo.py` 同样纳入自原示例；它定位 Isaac Sim 安装目录，并在转发给 Newton 前初始化 Warp runtime，使其能够从学习工程的 `demos` 目录启动。
- 原始目录 `isaac-sim-standalone-6.0.1-windows-x86_64/user_examples` 未被修改。
