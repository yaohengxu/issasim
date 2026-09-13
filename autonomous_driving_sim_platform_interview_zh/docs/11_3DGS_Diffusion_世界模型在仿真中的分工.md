# 3DGS、Diffusion、World Model：在智驾仿真中分别做什么

## 1. 先用一句话区分

```text
3DGS       ：这个已采集的世界“长什么样”，从新相机位姿看会是什么画面？
Diffusion  ：在指定条件下，还能生成哪些真实、多样的世界外观/场景/轨迹？
World Model：世界在时间和 action 作用下，“接下来会怎样演化”？
```

它们解决的维度不同，不能互相直接替代：

```text
                 静态/几何/外观                 时间/交互/因果
3DGS      强 ─────────────────────────────── 弱（原始形式通常不是动力学模型）
Diffusion  可生成多样外观/片段 ─────────────── 需专门时序/条件约束
World Model  可含观测表示 ──────────────────── 强：state + action → future

车辆物理、地图规则、碰撞、传感器标定、评测器：三者都不能替代。
```

## 2. 3DGS：真实场景的可渲染数字孪生外观层

### 2.1 它是什么

3D Gaussian Splatting（3DGS）从多视角照片/视频和相机标定出发，用大量带位置、形状、颜色/不透明度等属性的 3D Gaussian 表示场景，再通过 GPU splatting 快速渲染新视角。原始工作目标是高质量、实时的新视角合成，而非车辆动力学或交通行为建模。[3D Gaussian Splatting, SIGGRAPH 2023](https://arxiv.org/abs/2308.04079)

```text
多相机采集 + intrinsics/extrinsics
                 │
                 ▼
         3D Gaussian scene representation
                 │     输入：新相机 pose
                 ▼
          novel-view RGB render
```

### 2.2 它给智驾仿真带来的价值

| 用例 | 作用 | 适合解决的问题 |
| --- | --- | --- |
| 真实道路重建 | 用采集视频重建街景/路口外观 | 降低手工建模成本，缩小视觉背景域差 |
| Camera-in-the-loop | ego 位姿变化时渲染新相机视图 | 让视觉模型看到更接近真实的纹理/建筑/背景 |
| Real-to-sim 场景回放 | 日志场景转为可移动视角的数字孪生 | 从固定视频走向可执行的反事实测试 |
| 合成数据背景 | 与可编辑动态 actor、天气效果组合 | 增加背景多样性和真实感 |
| 可视化/回放 | 展示事故前后的真实场景结构 | 让调试和报告更直观 |

### 2.3 它不能天然解决什么

```text
3DGS scene
├─ 不等于 mesh / watertight geometry
├─ 不等于 collider，不能直接拿来可靠碰撞
├─ 不等于可编辑 HD map 或语义地图
├─ 不等于可控的动态车辆/行人行为
├─ 不等于 LiDAR/Radar 的物理回波模型
└─ 不等于随 action 演化的 world transition
```

例如 3DGS 可以让某个真实路口从新相机位姿看起来很像原场景，但“ego 左转后旁车是否减速”“轮胎是否打滑”“雷达是否发生多径”仍需交通/物理/传感器模型处理。

### 2.4 在仿真平台中的正确接法

```text
3DGS static/background layer
       │  RGB appearance
       ├──────────────> camera renderer / compositor
       │
HD map + semantic geometry ──> route / rules / evaluator
       │
physics collision mesh ──────> vehicle / actor dynamics
       │
dynamic actors / traffic model ─> pose updates per sim step
       │
       └──────────────> final sensor stream
```

即：将 3DGS 作为**视觉层/重建层**，保留独立的语义、碰撞和交通层。三套表示的坐标系必须对齐；不要因画面逼真而省略地图和 collider。

### 2.5 3DGS 的验证门槛

- 相机位姿偏移、视角变化、遮挡处的 RGB/几何是否稳定；
- 与真实相机的曝光、颜色、动态范围和畸变差异；
- 3DGS 坐标与 map、ego pose、dynamic actor 的对齐误差；
- 对下游感知的真实验证集收益，而不是只看 PSNR/视觉效果；
- 显存、加载、渲染延迟能否满足目标 camera FPS。

## 3. Diffusion：条件化的场景与观测多样性引擎

### 3.1 它是什么

扩散模型从噪声逐步去噪来学习数据分布，可在文本、地图、相机、布局、语义 mask、历史帧等条件下采样生成图像、视频、3D asset、轨迹或环境扰动。其本质是**概率生成**，不是物理求解器。基础 DDPM 工作展示了高质量数据生成能力。[Denoising Diffusion Probabilistic Models, NeurIPS 2020](https://proceedings.neurips.cc/paper/2020/hash/4c5bcfec8584af0d967f1ab10179ca4b-Abstract.html)

```text
condition: map / layout / weather / text / history / seed
                           │
noise ── iterative denoise ▼
                    generated sample
          RGB / video / texture / actor trajectory / scenario parameter
```

### 3.2 它在智驾仿真的作用

| 用例 | 输入条件 | 输出 | 平台价值 |
| --- | --- | --- | --- |
| 外观随机化 | 语义/深度/天气/时间 | RGB 风格、雨雾、光照、背景 | 弥补手工材质的外观覆盖 |
| 合成数据 | 3D layout、camera、class | 多样图像/视频/asset | 扩展稀有类别和长尾视觉数据 |
| 场景生成 | ODD bin、地图、事件约束 | actor 初始状态、事件参数、轨迹候选 | 自动扩增 corner case |
| 日志编辑 | 原始视频/场景条件 | 移除/添加/替换对象或天气 | 做受控反事实视觉实验 |
| 轨迹/行为先验 | 历史、地图、ego intent | 多模态 agent 行为候选 | 提高 NPC 行为多样性 |

### 3.3 最大风险：生成“看似合理”的无效样本

```text
单帧画面好看
    ≠ 多相机几何一致
    ≠ 多帧 object identity 一致
    ≠ 物体在正确道路/车道
    ≠ 标注、深度、速度可用
    ≠ ego action 后的因果响应正确
```

因此在 safety-critical 仿真中，Diffusion 的输出不可直接当真值。应放在生成候选层，后接约束与验证：

```text
Diffusion candidate
      │
      ├─ schema / seed / model version record
      ├─ map & lane constraint
      ├─ geometry / collision / kinematic validation
      ├─ multi-view and temporal consistency check
      ├─ semantic label / annotation validation
      └─ OOD / confidence / duplicate filtering
      ▼
accepted scenario or sensor augmentation
```

### 3.4 何时使用 renderer，何时使用 Diffusion

| 需求 | 规则渲染器更合适 | Diffusion 更合适 |
| --- | --- | --- |
| 精确相机几何、标注、深度 | 是 | 需额外约束，不能直接保证 |
| 控制每个物体 pose | 是 | 只能在强条件化后尝试 |
| 光照、材质、背景多样性 | 可做但资产成本高 | 常有优势 |
| 稀有外观/天气风格 | 需大量手工资产 | 可扩增候选分布 |
| 长时动作因果 | 不属于 renderer，交给 physics/traffic | 单独视频生成也不能保证 |

推荐把 renderer 作为结构/标签的主来源，把 Diffusion 用作经过校验的外观增强或场景候选；不要反向让 Diffusion 决定硬安全几何。

## 4. World Model：动作条件下的时间与交互层

### 4.1 它是什么

世界模型学习状态或观测的转移，典型形式为：

```text
state/history z(t), ego action a(t), map/context
                  │
                  ▼
world model Tθ
                  │
                  ▼
next z(t+1), agent trajectories, occupancy, observations, reward/risk
```

它可以在原始传感器、BEV/occupancy、结构化 actor state 或 latent 表示上工作。经典 World Models 工作展示了先学习压缩的时空表示，再在模型生成的环境中训练策略的思路。[World Models, Ha & Schmidhuber](https://arxiv.org/abs/1803.10122)

### 4.2 它在智驾仿真的作用

| 用例 | 作用 | 为什么比固定脚本强 |
| --- | --- | --- |
| 反应式 NPC | 预测其他交通参与者对 ego 行为的响应 | 不再是固定日志背景轨迹 |
| 反事实 roll-out | 比较多个候选 ego action 的未来风险 | 能回答“若提前刹车/并线会怎样” |
| 可学习交通行为 | 从真实日志拟合多样驾驶风格 | 比规则 agent 更贴近数据分布 |
| 低成本训练环境 | 在 latent/state 空间快速模拟许多步骤 | 减少高成本多传感器渲染 |
| 场景挖掘/扩增 | 对稀有交互采样可行未来 | 触达固定场景库之外的组合 |

### 4.3 不能把“预测得像日志”当成因果正确

```text
日志里的 future trajectory
      │  对应的是人类/原车实际采取的 action
      ▼
world model 若只拟合这个数据
      │
      └─ 不保证在“不同 ego action”下仍有正确反应
```

这叫反事实/因果挑战。世界模型在开环 ADE/FDE 上表现好，仍可能在 ego 激进并线、急刹、遮挡或 ODD 外输入时产生不合理 actor response。因此必须以动作条件闭环、交通规则、运动学、地图约束、不确定性估计和真实数据对齐来验证。

## 5. 三者怎样组合进一个仿真系统

```text
                      ScenarioSpec / ODD / seed
                                │
       ┌────────────────────────┼────────────────────────┐
       ▼                        ▼                        ▼
3DGS / renderer           Diffusion generator       World Model
真实背景与新视角           外观/事件/候选变体         action-conditioned actors
       │                        │                        │
       └─────────────── validity & alignment ────────────┘
                                │
                                ▼
      map + physics + traffic + sensor synchronization
                                │
                                ▼
          Camera/LiDAR/Radar → SUT → action → next state
                                │
                                ▼
              evaluator / replay manifest / real-data calibration
```

实用分工：

- 3DGS 提供真实道路的高质量可视外观；
- Diffusion 在明确条件下提出更多外观、对象或事件变体；
- World Model 让动态参与者/未来世界对 ego action 有反应；
- 地图、物理、规则、传感器和 evaluator 负责把三者约束成可执行、可度量、可复现的仿真。

## 6. 统一工程接口：不要把模型直接塞进仿真内核

```text
SceneAppearanceProvider.render(camera_pose, time) -> RGB/aux data
  ├─ 3DGS renderer
  └─ raster/RTX renderer

ScenarioGenerator.sample(odd_condition, seed) -> ScenarioCandidate
  ├─ Diffusion generator
  └─ rule-based generator

AgentTransition.predict(history, ego_action, map, seed) -> AgentStateDistribution
  ├─ World model
  └─ rule/IDM/behavior-tree NPC
```

每个 adapter 的输入输出必须写清：坐标系、时间轴、单位、条件、随机 seed、模型/资产版本、置信度和 latency。平台对候选输出做统一 validity filter，并让规则 baseline 与学习模型共享 evaluator。

## 7. 评测与上线门槛

| 技术 | 不应只看 | 至少还应看 |
| --- | --- | --- |
| 3DGS | PSNR、画面“像不像” | 相机标定/对齐、下游感知收益、时延、动态一致性 |
| Diffusion | FID、漂亮样例数 | 条件可控性、几何/时序/标签有效率、ODD 覆盖、真实下游收益 |
| World Model | 日志 ADE/FDE | action 条件反事实、规则/物理有效率、闭环安全、校准与不确定性 |

所有指标均应按 ODD 分桶，并与真实日志、shadow mode 或真机数据做校准。模型的“视觉质量”或“预测平均误差”只能是中间指标，不能单独作为安全结论。

## 8. 面试 60 秒答案

> 3DGS、Diffusion 和 World Model 分别强化仿真的不同层。3DGS 从真实多视角采集重建可快速渲染的新视角，适合真实道路数字孪生的视觉背景，但它不是 collider、HD map 或车辆动力学。Diffusion 用条件生成扩增天气、外观、场景事件或行为候选，主要解决多样性和长尾覆盖，但输出必须经过几何、地图、时序和标注校验。World Model 学习状态/观测在 ego action 下的时间转移，适合反应式 NPC、反事实推演和低成本 roll-out，但不能只以日志轨迹误差证明因果可信。平台应将三者包在外观、场景生成、agent transition 的 adapter 后，与物理、地图、传感器同步和统一 evaluator 组合；所有输出都要记录版本和 seed，并以闭环安全及真实数据对齐作为最终门槛。

## 9. 3DGS 系统教程：从场景采集到智驾仿真接入

### 9.1 3DGS 的核心直觉

传统三角网格用大量顶点和面表示物体表面；NeRF 类方法用神经网络隐式表示空间中的辐射场；3DGS 则用大量可微的三维椭球 Gaussian 近似场景中的可见外观。

```text
一个 Gaussian i 的典型属性

μ_i       ：三维中心位置 (x, y, z)
Σ_i       ：三维协方差（常用 rotation + scale 表示椭球形状）
α_i       ：不透明度 / density
c_i(view) ：颜色，可随观察方向变化（常使用球谐 SH）

场景 = {Gaussian_1, Gaussian_2, ..., Gaussian_N}
```

从某个相机看过去，每个 3D Gaussian 会投影为屏幕上的 2D 椭圆。渲染器按深度排序，并以 alpha compositing 混合颜色：

```text
3D Gaussian cloud
       │ camera pose + intrinsics
       ▼
project to 2D ellipses
       │ depth-aware sorting
       ▼
alpha compositing / splatting
       ▼
RGB image (and implementation-specific auxiliary buffers)
```

它的强项是：高密度的真实外观可以由 GPU rasterization 高效渲染，因此很适合“从采集过的真实路口，生成相机稍微移动后的真实感画面”。原始 3DGS 工作从稀疏相机标定点出发，优化 anisotropic Gaussian，并使用可见性相关的快速渲染。[3D Gaussian Splatting, SIGGRAPH 2023](https://arxiv.org/abs/2308.04079)

### 9.2 3DGS 不等于点云

| 表示 | 主用途 | 能否直接高质量新视角渲染 | 能否直接做碰撞/物理 |
| --- | --- | --- | --- |
| LiDAR point cloud | 测距、几何感知 | 通常不适合 photo-realistic RGB | 否，需重建/近似 |
| Mesh + texture | 几何、渲染、碰撞 | 可以 | 可以，但取决于 collider 设计 |
| 3DGS | 外观重建、新视角渲染 | 是，主要优势 | 否，需独立几何/collider |
| NeRF/radiance field | 外观重建、新视角渲染 | 可以，常较重 | 否，需独立几何/collider |

3DGS 中的 Gaussian 不是传感器实际测得的离散 LiDAR 点，也不是 watertight mesh 表面。它表达的是“从观察视角能呈现什么外观”，不要将它误用为安全距离、道路边界或碰撞体。

### 9.3 训练/重建需要哪些输入

一个可用道路 3DGS 的基本输入是：

```text
采集图像/视频
   + 每张图像的相机内参 K（焦距、主点、畸变处理方式）
   + 每帧相机外参 T_world_camera 或等价 ego pose
   + 足够的视角覆盖、重叠和曝光质量
   + 时间/坐标基准
   + 可选：深度、语义 mask、动态物体 mask、HD map 对齐
```

常见离线流程：

```text
1. 数据清洗
   去模糊、坏帧、时间错位；处理隐私、版权、车牌/人脸策略。
2. 标定与位姿
   相机内参、外参、ego trajectory；可经 SfM/SLAM/里程计融合获得。
3. 初始化
   从稀疏重建点/点云初始化 Gaussian 的中心、颜色和尺度。
4. 优化
   对比训练视图与 render，优化位置、形状、颜色、透明度等参数。
5. densify / prune
   在误差大处增加 Gaussian，删除贡献小或异常的 Gaussian。
6. 验证与导出
   在未训练相机位姿评估 render；导出版本化 asset 与坐标转换。
```

采集轨迹只有“沿道路的一条前向车道线”时，横向/大视角变化的质量会较差；相机没拍到的区域不能凭空提供可靠几何。这是新视角合成的覆盖边界，不是训练参数能完全补救的问题。

### 9.4 坐标系：智驾接入最容易犯错的部分

在仿真中至少要统一下列坐标：

```text
W_map        ：地图/全局坐标
W_3dgs       ：3DGS 重建坐标
E_ego(t)     ：自车坐标，随时间变化
C_front(t)   ：前视相机坐标
S_sim        ：仿真器世界坐标

需要固定且可测试的变换：
T_S_sim_W_map, T_W_map_W_3dgs, T_E_ego_C_front
```

```text
P_camera = T_camera_ego · T_ego_map(t) · T_map_3dgs · P_3dgs
```

工程要求：函数名要写出方向（如 `T_camera_map`），配置中记录四元数顺序、米/厘米、左/右手系、坐标轴约定。使用已知路标、车道线或标定板做投影单元测试：同一个世界点投到真实图像和 3DGS 渲染图中，像素误差应可量化，而不是“肉眼大致重合”。

### 9.5 静态、动态与可编辑性边界

```text
静态背景：建筑、道路、路牌、树、固定护栏
  → 最适合用 3DGS 重建和渲染。

动态对象：ego、车辆、行人、骑行者、信号灯状态、施工物
  → 通常用独立可编辑 mesh/asset + pose + animation/physics。

环境变化：雨雪、昼夜、施工、停车位变化
  → 可用 renderer 参数、Diffusion、替换资产或多版本 3DGS；必须保留配置。
```

若采集时道路上的车辆被“烘焙”进 static 3DGS，闭环时可能出现 ghost car、错误遮挡或与可编辑 actor 重叠。常见处理是训练前用动态 mask 去除/弱化动态对象，或把静态背景和动态对象分层；无论哪种，都要在 manifest 中记录处理策略和已知残留。

### 9.6 Camera-in-the-loop 的接入时序

```text
sim time t
  │
  ├─ physics/traffic 更新 ego 与 dynamic actor pose
  ├─ 计算 camera pose：T_world_camera(t)
  ├─ 3DGS render static background
  ├─ raster/RTX render dynamic actors + shadows/occlusion（按系统设计）
  ├─ compositing / camera noise / exposure / latency
  ├─ 发布 RGB frame(timestamp=t_capture)
  └─ 感知模型完成推理后，发布 result(timestamp=t_ready)
```

闭环场景中不能把 3DGS 只当作一段固定视频。ego 转向或变道后，必须使用新的相机位姿重新渲染；同时要模拟相机采集、编码、传输和 DNN 推理之间的延迟，否则 planner 会错误地获得“未来/零延迟”观测。

### 9.7 与动态 actor 的合成要求

将 3DGS 背景和 mesh actor 直接做简单图片叠加容易出错。至少需要：

- 统一的 camera intrinsics 和 world-to-camera transform；
- 深度/可见性顺序，确保车辆经过路牌/护栏时正确遮挡；
- 光照、阴影、曝光、色调一致性，避免 actor 像贴图；
- 动态 actor 在 map 上可行驶、与 3DGS 道路几何对齐；
- 对重建里已存在的静态车/行人残影做 mask 或剔除。

若需要可靠的 semantic/depth/instance 标注，优先让动态 actor 从结构化渲染器产生真值；不能仅从最终合成 RGB 反推标注。

### 9.8 传感器边界：RGB 是强项，其他传感器要谨慎

| 数据需求 | 3DGS 是否可作为主来源 | 原因 |
| --- | --- | --- |
| RGB camera | 可以，特别适合静态真实背景 | 正是新视角外观渲染问题 |
| 深度 | 可输出估计/渲染辅助，但须单独验证 | 外观表示不天然等于已校准深度传感器模型 |
| 语义/实例 mask | 需额外 3D 语义或结构化 actor 层 | 原始 3DGS 不天然提供可靠类别/实例 |
| LiDAR | 不建议直接替代物理 ray casting | beam pattern、反射率、遮挡、range noise 不等价 |
| Radar | 不建议直接替代雷达模型 | RCS、多径、Doppler、虚警不是 RGB 外观问题 |

因此，智驾平台常采用“3DGS 供应 camera 背景 + 专用传感器模型供应 LiDAR/Radar + 共享 map/actor 状态”的混合架构。若有人声称“接入 3DGS 后多传感器问题都解决了”，应追问其雷达和 LiDAR 的标定与物理模型。

### 9.9 工程接口示例

```python
class ThreeDGSAppearanceProvider:
    def load(self, asset_uri: str, map_to_3dgs: "Transform") -> None:
        """加载版本化 3DGS asset；登记坐标转换和相机模型兼容信息。"""

    def render(self, capture_time_ns: int, camera: "CameraSpec", T_map_camera: "Transform") -> "RgbFrame":
        """仅输出静态/背景 RGB；frame 必须携带时间戳、pose、intrinsics、asset version。"""

    def health(self) -> "RenderHealth":
        """返回 GPU memory、render latency、missing-region/coverage 等运行状态。"""
```

```text
CameraSpec      = intrinsics + distortion model + resolution + exposure config
RgbFrame        = pixels + timestamp + frame_id + T_map_camera + provenance
RenderHealth    = p50/p95 latency + GPU memory + asset version + warning flags
```

关键是 provider **不负责**交通规则、车辆动力学、碰撞和最终评测；这些职责留给场景/物理/评测模块。这样可替换成 RTX renderer、纯 raster renderer 或 3DGS，而不改变被测 AD stack 的输入合同。

### 9.10 评测：从图像指标走到下游效用

```text
Level 1：render fidelity
  新视角 RGB、边界、遮挡、曝光是否接近保留真实图像？

Level 2：geometric alignment
  车道线、路标、静态物体与 map/pose 是否对齐？

Level 3：sensor-system effect
  同一感知模型在真实数据、传统 renderer、3DGS 背景上的检测/占据表现如何？

Level 4：closed-loop utility
  3DGS 场景中的规划安全趋势，是否和真实/shadow 场景一致？
```

建议指标：

- **Novel-view image metric**：PSNR/SSIM/LPIPS 可辅助检查外观，但不能单独代表智驾可用性；
- **Reprojection error**：已知 3D/map 点重投影到图像的像素误差，检验坐标对齐；
- **Semantic consistency**：路牌、车道、信号灯等关键语义是否稳定；
- **Downstream delta**：同一感知/规划模型在真实与 3DGS 条件下的性能差；
- **Runtime**：首帧加载、p50/p95 render latency、显存、丢帧率；
- **Coverage**：相机轨迹是否处于采集覆盖范围，超范围渲染要标为低置信而非静默使用。

### 9.11 常见失败模式与排查顺序

```text
画面“漂”/车道线不重合
  → 先查 T_map_3dgs、单位、坐标轴、外参，不要先调渲染参数。

转弯后突然糊/空洞/闪烁
  → 检查采集视角覆盖、动态物体残留、曝光变化、Gaussian 密度与可见性。

动态车像贴上去/遮挡错误
  → 检查 depth composition、相机内参、actor pose、背景中 ghost actor。

闭环结果虚高
  → 检查 camera latency、是否使用了未来 pose、场景是否只覆盖训练路线、是否存在 sim shortcut。

GPU 爆显存或 p95 延迟过高
  → 检查 asset 分块/LOD、加载策略、分辨率、Gaussian 数量、缓存和并发 worker 数。
```

### 9.12 3DGS 项目的最小交付物

```text
assets/
  intersection_001_3dgs/        # Gaussian asset、版本、hash、许可证
  calibration/                  # camera intrinsics/extrinsics、坐标转换
  masks/                        # dynamic-object/隐私/语义处理记录
configs/
  camera_front.yaml
  scenario_intersection_001.yaml
src/
  appearance_provider.py
  compositing_adapter.py
  validation.py
reports/
  coverage_alignment_runtime.md
  known_limitations.md
```

面试时能拿出上述 manifest、对齐测试和已知限制，比仅展示一段“真实感路口视频”更符合仿真平台岗位期待。

### 9.13 3DGS 面试 60 秒答案

> 3DGS 是从多视角标定图像学习的显式 Gaussian 场景表示，输入新相机位姿后可做高质量新视角渲染。在智驾仿真中，我把它定位为真实道路数字孪生的视觉背景层：它能降低手工资产成本，并让 camera-in-the-loop 看到更接近真实的建筑、道路纹理和静态环境。它不等于 mesh、collider、HD map 或 LiDAR/Radar 物理模型，因此我会将它与独立的地图语义层、碰撞几何、动态 actor、车辆/交通模型组合。接入时最关键的是 map、3DGS、ego 和 camera 坐标系对齐，以及按 sim time 重新渲染、记录 sensor latency；评测也不能只看 PSNR，而要看重投影误差、下游感知差异、闭环行为趋势、render 延迟和采集覆盖边界。

## 10. World Model 系统教程：在自驾仿真业务中解决什么问题

### 10.1 自驾里的 World Model 不是一个固定模型名

在自驾业务中，World Model 是一类能力：在给定历史、地图、交通参与者状态和 ego action 时，学习或估计未来世界的概率分布。它可以工作在不同表示空间：

```text
Raw sensor world model
  camera/lidar history + action → future RGB / point cloud / token

Structured traffic world model
  map + actor states + ego action → future actor trajectories / occupancy

Latent world model
  encoded history z(t) + action → z(t+1), reward/risk/termination

Hybrid world model
  learned agent behavior + rule/map/physics constraints → executable world state
```

自驾仿真最实用的往往是后两类：不必完整“幻想”所有像素，而是先让交通参与者对 ego action 产生合理反应，再用物理/传感器渲染器生成后续观测。

### 10.2 它要解决的五个业务问题

#### 问题 A：日志回放没有反事实和交互

真实日志只记录了一个已发生的世界：原车当时选择了 action `a_log`，周车也对那个真实行为作出了反应。若被测 ADS 选择另一条轨迹 `a_ads`，日志无法告诉我们后续会发生什么。

```text
历史日志
  ego action = a_log  ──> recorded future

要验证的 ADS
  ego action = a_ads  ──> ?
```

**World Model 的作用**：以 `history + map + a_ads` 为条件，生成多个可能的其他参与者未来，从而让仿真能评估“如果 ego 提前并线/急刹/让行，周车会如何响应”。

#### 问题 B：手工规则 NPC 覆盖有限且行为生硬

传统 IDM、跟车规则、行为树和脚本场景可解释、稳定，但很难穷举各城市、驾驶风格、博弈、犹豫、礼让与长尾交互。

**World Model 的作用**：从真实日志学习速度、让行、插队、合流、横穿、跟车等行为分布，为 NPC 提供多模态、条件化的行为候选。规则仍负责最低安全边界和 ODD 约束，World Model 负责提高真实分布覆盖。

#### 问题 C：高保真传感器闭环太昂贵

多相机、LiDAR、Radar 的全量渲染和端到端模型推理成本高，RL 或场景搜索需要大量 trial-and-error。

**World Model 的作用**：在结构化 state/occupancy/latent 空间快速 rollout，用低成本近似筛掉明显无价值场景和 action；只把高风险、高信息量候选送到高保真物理+渲染仿真复核。

```text
大量 action/scenario candidates
            │
            ▼
fast world-model rollout / risk ranking
            │  选出高风险、分歧大、OOD 候选
            ▼
high-fidelity simulator + sensor-in-the-loop validation
```

#### 问题 D：长尾场景库难以系统扩增

只靠人工编写场景，数量大但组合有限；只靠随机采样，则大量样本没有风险价值。

**World Model 的作用**：在 ODD 条件下采样“合理但不同”的未来，或搜索能暴露 SUT 失败的初始状态/他车行为；之后通过地图、运动学、规则和碰撞校验，形成可版本化的新场景回归集。

#### 问题 E：端到端/RL 策略需要动作后果信号

端到端策略和 RL 的核心不是“模仿日志轨迹”，而是学习在动作改变世界后如何恢复和优化。只有 `T(s, a) → s'` 这一转移可用，策略才能比较多个 action 的长期风险/收益。

**World Model 的作用**：为 model-based RL、MPC、行为规划或 counterfactual evaluator 提供快速的未来推演；但最终安全评价仍须用高保真仿真或真实数据锚定。

### 10.3 一张完整业务链路图

```text
真实日志 / 采集数据
  ├─ map、route、traffic light、actor track
  ├─ ego action / CAN / pose
  └─ camera/lidar/radar history
            │
            ▼
    Offline data pipeline
  同步、坐标统一、匿名化、质量筛选、ODD 标签、版本化
            │
            ▼
    World Model training / calibration
  history + map + ego action → future state distribution
            │
            ▼
    Online simulation adapter
  predict / sample / confidence / uncertainty / latency
            │
            ▼
    Validity & safety filter
  map、碰撞、运动学、交通规则、OOD、置信度、fallback
            │
            ▼
    Sim Core
  physics + traffic + sensor render + SUT action feedback
            │
            ▼
    evaluator / replay manifest / real-data calibration
```

### 10.4 模型输入、输出与动作条件必须明确

一个可接入平台的 traffic World Model 不应只写成“输入视频，输出视频”。要定义合同：

```text
Input
  history window: actor poses/velocities/heading, occupancy, traffic lights
  static context: lane graph, drivable area, route, speed limit
  ego condition : executed/plan action, vehicle footprint, intent
  random state  : seed, sampling temperature, scenario id

Output
  per-agent trajectory distribution / occupancy flow / existence probability
  uncertainty/confidence and model latency
  provenance: checkpoint, feature schema, normalization, seed
```

每个 actor 输出至少需要：`id`、类别、时间戳、pose、velocity、尺寸、存在概率和协方差/多模态候选。没有时间戳、坐标系、单位与不确定性，后续仿真器无法安全地消费模型结果。

### 10.5 它如何“解决”：混合式 transition，而非全盘接管

```text
              World Model proposes distribution
                         │
                         ▼
       sample / select candidate NPC next states
                         │
          ┌──────────────┼────────────────┐
          ▼              ▼                ▼
      Map check      Kinematic check    Rule check
   lane/drivable     speed/accel/yaw    light/right-of-way
          │              │                │
          └──────────────┴────────────────┘
                         │
                         ▼
               Physics / collision resolver
                         │
                         ▼
         accepted state or fallback NPC policy
```

这解决的不是“世界模型永远正确”，而是：

- 用模型提供数据驱动的**候选行为分布**；
- 用地图、规则、动力学和碰撞做**硬约束**；
- 用不确定性/置信度决定是否采样、重试或降级；
- 用规则 NPC 或保守行为作为**fallback**；
- 用统一 evaluator 判断对 SUT 的业务价值。

这是比“把生成模型直接当仿真器”更适合安全关键工程的做法。

### 10.6 训练与数据：为什么仅有日志不够

```text
日志数据提供：在真实驾驶行为 a_log 下发生的 future
需求是：       在不同候选行为 a_ads 下会发生的 future
```

因此单纯最小化日志上的 ADE/FDE，容易学成“复述人类行为”，却不理解 ego 不同行为的因果后果。工程上可采用的缓解措施包括：

1. **Action-conditioned training**：明确将 ego 历史和计划/action 输入 transition；
2. **扰动数据**：从专家轨迹周围采样横向/纵向/时序扰动，训练恢复和反应；
3. **交互数据覆盖**：刻意保留并线、让行、急刹、路口、遮挡等交互片段并按 ODD 分桶；
4. **规则/物理约束**：将地图、可行驶区、速度/曲率、碰撞等约束加入训练或后处理；
5. **不确定性估计**：输出多模态分布/ensemble，而不是单条“平均轨迹”；
6. **高保真复核**：将模型 rollout 与规则/物理仿真、真实 held-out 片段对照。

不能从观察数据中无条件保证所有反事实因果关系；面试时应明确这是数据覆盖和因果识别的限制，而非承诺 World Model 可以“预测一切”。

### 10.7 开环指标与闭环指标分别回答什么

| 层次 | 例子 | 能回答 | 不能单独证明 |
| --- | --- | --- | --- |
| 开环轨迹 | ADE/FDE、速度误差、occupancy IoU | 是否贴近已发生的日志 future | 对不同 ego action 的反事实是否正确 |
| 开环概率 | NLL、calibration、coverage、多样性 | 分布是否覆盖日志多模态未来 | 采样行为是否合规、安全、可执行 |
| 有效性过滤 | 碰撞率、越界率、规则违规率 | 输出是否进入基本可行集合 | 对 SUT 是否有真实业务价值 |
| 闭环交互 | action-conditioned response、TTC、collision | ego 与 NPC 交互是否安全/合理 | 与现实是否完全等价 |
| 下游效用 | SUT failure discovery、真实验证增益 | 是否改善场景覆盖/找出问题 | 模型本身所有方面正确 |

### 10.8 一个具体业务例子：匝道合流

```text
Scenario
  ego 要从匝道并入主路；主路后车距离与速度存在不确定性。

固定日志背景
  后车按原日志轨迹走；只能测 ego 对该一条未来的应对。

World Model in loop
  输入后车历史、车道拓扑、ego 并线意图/轨迹。
  输出：后车减速让行 / 保持速度 / 加速封堵 等多个概率候选。
  经过速度、加速度、车道、碰撞、规则检查后，进入仿真。

Evaluator
  merge success、最小间距、TTC、急刹、交通阻塞、规则合规、失败模式。
```

这让平台从“ego 是否能复述人类一次并线”升级为“ego 面对多种合理他车反应时是否安全、稳健”。

### 10.9 平台接口与伪代码

```python
class AgentTransitionProvider:
    def reset(self, scenario: "ScenarioSpec", seed: int) -> None:
        """固定 checkpoint、归一化配置、随机状态与 fallback 策略。"""

    def predict(
        self,
        history: "WorldHistory",
        ego_action: "EgoAction",
        map_context: "MapContext",
        horizon_s: float,
    ) -> "AgentStateDistribution":
        """输出多 agent、多模态未来及不确定性；不直接修改仿真世界。"""

    def validate_and_sample(
        self, prediction: "AgentStateDistribution", constraints: "ConstraintSet"
    ) -> "AcceptedAgentStates":
        """执行地图、运动学、规则、碰撞、OOD 与置信度过滤；必要时 fallback。"""
```

主循环中要明确顺序：

```text
SUT reads observation(t)
       → outputs ego action(t)
       → World Model predicts NPC distribution conditioned on action(t)
       → filter/sample/fallback
       → simulator applies vehicle + NPC transition
       → sensors render observation(t+1)
```

不要让 provider 直接绕过仿真器 teleport ego 或写入评测结果；它只提供动态世界的候选转移。

### 10.10 工程风险、治理和降级策略

| 风险 | 可能后果 | 平台防护 |
| --- | --- | --- |
| OOD 输入 | 输出荒谬轨迹、置信度虚高 | OOD detector、场景范围、fallback NPC |
| 模式坍塌/均值轨迹 | 交互单一、错过危险行为 | 多模态采样、diversity/coverage 指标 |
| 非因果拟合 | ego action 改变后响应不合理 | action-conditioned 闭环测试、反事实基准 |
| 不满足规则/物理 | 穿车、逆行、超大加速度 | constraint filter、collision resolver |
| 不可复现 | 同一场景结果漂移 | checkpoint/config/seed/采样参数写入 manifest |
| 过慢/成本高 | 无法批量运行 | latent rollout 分层筛选、缓存、超时、降级 |
| 数据泄漏/隐私 | 训练/评测不可信或合规风险 | 时间/地理/车辆级切分、数据治理、脱敏 |

### 10.11 面试 90 秒答案

> 自驾仿真中的 World Model 不是泛指生成视频，而是学习世界在地图、历史和 ego action 条件下的时间转移。它主要解决三类业务问题：第一，真实日志只能回放原车实际行为，不能回答不同 ego action 下周车会怎样响应；第二，手工 NPC 规则难以覆盖真实多样交互；第三，全量传感器闭环和 RL 搜索成本高。我的做法是把 World Model 放在 agent transition 层：它根据历史、地图和 ego action 输出多个 NPC/occupancy 未来及不确定性，而不是直接改写仿真世界。随后由地图、运动学、交通规则、碰撞和 OOD filter 校验，失败则回退到规则 NPC；仿真器再推进并重新生成传感器。评测上我会先看开环轨迹/概率指标，再看 action-conditioned 闭环的 collision、TTC、合流成功、规则合规和下游失败发现率，并记录 checkpoint、seed、条件和版本。这样 World Model 增强交互真实性和场景覆盖，但不会被当成物理与安全评测的替代品。
