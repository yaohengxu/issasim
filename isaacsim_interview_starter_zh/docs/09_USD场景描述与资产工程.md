# USD 场景描述与资产工程

## 1. 用“可组合场景图”理解 USD

```text
/World                         Stage 根
├─ /PhysicsScene                重力、求解配置
├─ /Environment
│  ├─ /Table                    静态碰撞 + 可视 mesh
│  └─ /Lights
├─ /Robots
│  └─ /Franka                   reference 到机器人资产
├─ /Objects
│  └─ /Mug_01                  visual + collider + rigid body + material
├─ /Sensors
│  └─ /FrontCamera              内外参、渲染产品/标注
└─ /Task
   ├─ /Targets
   └─ /DebugMarkers
```

每个节点是 Prim。Prim 的类型和 schema 决定意义：`Xform` 管层级位姿，`Mesh/Cube/Sphere` 管几何，Physics API 赋予碰撞与质量，Camera schema 表示相机。路径是工程接口的一部分，控制代码、标注工具和测试都依赖它；不要随意改路径却不更新配置。

## 2. USD 组合（composition）是工程价值所在

```text
robot_base.usd       table_asset.usd        mug_asset.usd
      │                    │                    │
      └──── reference / payload / variant ──────┘
                               │
                       tabletop_base.usda
                               │  subLayer / override
                        task_pick_mug.usda
                               │  session layer（临时调参）
                        当前打开的 Stage
```

- **reference**：在场景中引用完整资产，保留资产自身层级。
- **payload**：可延迟加载的大资产，适用于大场景内存管理。
- **sublayer**：叠加一整层编辑。
- **override (`over`)**：只覆写引用资产的某些属性，例如颜色、初始 pose。
- **variant set**：同资产可切换网格精度、夹爪型号、材质版本。
- **session layer**：交互时的临时编辑，通常不应当作可复现任务配置唯一来源。

面试表达：把机器人本体、环境资产、任务初始状态、随机化参数分层，可以防止任务脚本修改污染基础资产，也方便多个任务复用同一机器人。

## 3. 一个最小 `.usda` 文件

请打开 [`assets/tabletop_task_template.usda`](../assets/tabletop_task_template.usda)。它是文本格式 USD，演示了 Stage 元数据、物理场景、静态桌面、动态目标物和视觉标记。关键片段如下：

```usda
#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Z"
)
def Xform "World" {
    def Cube "TableTop" (prepend apiSchemas = ["PhysicsCollisionAPI"]) { ... }
    def Sphere "Target" (
        prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsRigidBodyAPI", "PhysicsMassAPI"]
    ) { ... }
}
```

**视觉几何、碰撞几何、动力学属性可以是不同 Prim**。生产场景常让低面数 collision mesh 与高面数 visual mesh 分离，避免把昂贵的视觉网格直接送给碰撞求解器。

## 4. 桌面抓取场景的分层建议

```text
assets/
  robots/franka.usd             # 不在任务内直接编辑
  objects/mug.usd
  environments/table.usd
scenes/
  tabletop_base.usda            # 灯光、相机、桌面、物理全局设置
  pick_mug_scene.usda           # reference + 初始位姿
tasks/
  pick_mug.yaml                 # seed、目标、成功阈值、随机化范围
  pick_mug.py                   # 状态机/奖励/控制接入
```

推荐把“可版本化的可变参数”放 YAML/JSON 或自定义配置，而不是把每轮随机位置保存回基础 USD。USD 保存空间结构与资产关系，配置保存实验意图，两者职责不同。

## 5. 位姿、单位和语义

- 明确 `metersPerUnit` 和 `upAxis`；机器人控制和物理通常按米、Z-up 解释。
- 明确 Prim 的父子层级；子 Prim 的局部坐标不是世界坐标。
- 命名要稳定，如 `/World/Robots/Franka`、`/World/Objects/Target`，不使用 GUI 自动编号作为代码契约。
- 为合成数据分配 semantic label / class，区分实例 ID 与语义类别；标签版本同资产版本一起管理。

## 6. 资产审查清单

| 维度 | 必查项 | 失败后现象 |
| --- | --- | --- |
| 尺寸 | mesh 单位、AABB、原点 | 物体像巨人/蚂蚁，重力和碰撞异常 |
| 坐标 | up axis、前向轴、pivot | 导入后躺倒、抓取 pose 错 |
| 视觉 | UV、法线、材质、LOD | 渲染域差、训练数据失真 |
| 物理 | collider、质量、惯量、摩擦 | 穿透、弹飞、抓不稳 |
| 语义 | 类别、实例、版本 | 标注错类、数据不可追溯 |
| 许可 | 来源、再分发限制 | 无法合规交付 |

## 7. 面试追问：为什么不用一个巨大 USD？

因为大而扁平的场景不利于复用、diff、按需加载和任务变体。正确做法是将相对稳定的资产封装，将任务特定修改以 layer/reference/variant 组合；但层数也不能无限堆叠，应有清楚的 owner 和命名规范，避免 composition arc 难以排查。
