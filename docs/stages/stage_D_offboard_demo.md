# Stage D V1 — 感知参与的 Offboard 自动切割 Demo

## 1. 阶段目标

Stage D V1 的目标不是一次解决完整的自主树障处理问题，而是先闭合：

```text
RGB + LiDAR 感知
      ↓
BranchModel (world)
      ↓
PX4 Offboard：UAV XYZ + Yaw
      ↓
固定机械臂 PREWORK/CUT 姿态
      ↓
链锯切割接触 / cut_progress / detach
      ↓
安全撤离 / HOME / 降落
```

核心原则：**先闭合完整系统链路，再逐层解除简化约束。**

---

## 2. V1 的问题拆解

当前系统有两大复杂问题：

1. 感知：全局搜索、识别、跟踪、多视角建模、目标选择；
2. 操作：机械臂规划、碰撞、可达性、安全裕度、UAV-机械臂耦合。

V1 分别做如下简化。

### 2.1 感知侧

保留 Stage C 已完成的 RGB-LiDAR 感知，不绕开感知：

- 人工提供树木粗略位置 `tree_hint_x/y`；
- 人工给定合适观察高度；
- 不做全局上下搜索、多树枝分类和复杂跟踪；
- UAV 到达观察区后执行小弧线多视角观察；
- 精确树枝 `center / direction / radius / length` 仍由 `/perception/branch_model` 在线获得；
- V1 切割点直接定义为 `BranchModel.center`。

### 2.2 操作侧

V1 暂时不做机械臂主动进刀和复杂规划：

- 起飞、降落、远距离移动：机械臂 `HOME`；
- 到达 `FAR_APPROACH` 后：机械臂展开到 `PREWORK`；
- V1 中 `PREWORK == CUT_POSE`；
- 最终进刀由 UAV 沿机头/工具方向低速前进完成；
- UAV 的 XYZ + Yaw 暂时承担“大范围机械臂自由度”的作用。

后续再将 UAV 进刀替换/扩展为 `movetcp` 主动进刀、MoveIt 2、碰撞规划等。

---

## 3. Live Target 与 Locked Target

`/perception/branch_model` 当前输出位于 `world` 坐标系。

V1 将目标分成两类：

```text
OBSERVE / ARC_SCAN
    ↓
Live BranchModel 持续更新
    ↓
质量与稳定性确认
    ↓
TARGET_LOCK
    ↓
Locked Target (world)
```

Locked Target 保存：

- `center_world`
- `direction_world`
- `length`
- `radius`

锁定后，UAV 可以上升至链锯工作高度。即使树枝离开 RGB + LiDAR 联合工作区，后续 FAR / NEAR / CUT 仍使用 world 中冻结的目标。

只有 `reset_target / reobserve` 才主动清除 Locked Target。

---

## 4. 观察策略

已实测舒适观察状态：

```text
QGC 相对高度约 3.0 m
Gazebo base_link world z 约 3.22 m
Branch center 约 [4.07, 0.08, 3.02] m
```

V1 参数初值：

```text
tree_hint        ≈ [4.0, 0.1] m
observe_distance ≈ 1.4 m
observe_height   ≈ 起飞点上方 3.0 m
arc              ≈ ±25 deg
```

观察过程：

```text
OBSERVE CENTER
    ↓
LEFT VIEW
    ↓
RIGHT VIEW
    ↓
CENTER VIEW
    ↓
TARGET_LOCK
```

全程 Yaw 朝向树木/目标区域。

---

## 5. 接近与切割几何

V1 使用三级接近：

```text
FAR_APPROACH
↓
NEAR_APPROACH
↓
CUT_IN
```

最终进刀方向由当前接近侧指向 Locked Target，并自动生成目标 Yaw。

> V1 暂不使用 `BranchModel.direction` 优化切割平面；`direction` 会被锁定并保留，后续用于“垂直枝条主轴切入”、切点优化和联合规划。

FAR / NEAR 均沿最终进刀方向的反方向退距离生成，而不是写死 world X/Y。

固定 PREWORK 状态下，当前链锯中心相对 `base_link` 约：

```text
[+0.156, +0.030, -0.455] m
```

链锯外侧有效切割区再向前约 0.036 m，因此 V1 使用可调初值：

```text
tool_forward_offset ≈ 0.19 m
cut_z_offset         ≈ +0.455 m
```

这些值是 Demo 的几何补偿参数，不视为最终高精度 TCP 标定。

---

## 6. 切割策略

```text
NEAR_APPROACH
↓
Saw = 1000 rpm
↓
确认 saw_actual_rpm 达标
↓
CUT_IN：UAV 低速前进
↓
target_contact == true
↓
冻结当前位置 / CUT_WAIT
↓
cut_progress 增长
↓
cut_success / detach
```

最终成功依据是现有 `auto_cut_controller` 触发树枝 detach，并同步发布 `/lumberjack_arm/cut_success`。

V1 同时设置两类硬限制：

- `cut_in_max_distance`
- `cut_timeout_sec`

若失败：

```text
Saw OFF
↓
自动反向退出到 NEAR
↓
CUT_FAILED
↓
等待人工检查 / QGC 接管
```

不做无限自动重试。

---

## 7. 切割后的撤离顺序

不在锯刚完成切割时立即折叠机械臂：

```text
CUT SUCCESS
↓
Saw OFF
↓
反向退出到 NEAR
↓
继续退出到 FAR
↓
Arm -> HOME
↓
返回起飞点上空
↓
LAND
```

这样避免机械臂在树枝附近折叠时发生额外碰撞。

---

## 8. 自动模式与分阶段调试模式

### 完整演示

```text
start
```

完整链：

```text
ARM HOME
→ OFFBOARD
→ TAKEOFF
→ OBSERVE
→ ARC_SCAN
→ TARGET_LOCK
→ FAR_APPROACH
→ ARM PREWORK
→ NEAR_APPROACH
→ SAW_SPINUP
→ CUT_IN
→ CUT_WAIT
→ RETREAT_NEAR
→ RETREAT_FAR
→ ARM HOME
→ RETURN_HOME
→ LAND
```

### 分阶段调试

```text
takeoff
observe
scan
lock
far
prework
near
cut
retreat
armhome
land
```

同时保留：

```text
status
pause
resume
retry
reset
reset_target
abort
params
```

---

## 9. 人工接管与恢复

研发 Demo 必须允许人工介入，而不是封闭黑盒。

```text
pause
↓
Saw OFF
↓
PX4 切回 Position Mode
↓
QGC 手动调整位置 / Yaw
↓
resume
↓
重新读取当前 UAV 位姿
↓
重新进入 OFFBOARD
↓
重新计算当前阶段目标
```

若在 `SAW_SPINUP / CUT_IN / CUT_WAIT` 中暂停，恢复时不从半截刀路继续，而是重新从 `NEAR_APPROACH` 建立安全状态。

如果 PX4 意外退出 OFFBOARD，任务节点也会检测并自动进入 PAUSED 逻辑。

---

## 10. 坐标系原则

任务几何统一在 Gazebo/ROS `world` 中完成：

- BranchModel：`world`
- UAV 当前位姿：Gazebo `world`
- FAR / NEAR / CUT：`world`

只在 PX4 输出边界统一转换为 NED `TrajectorySetpoint`。

任务节点启动时使用 Gazebo odometry 与 PX4 VehicleOdometry 自动标定固定的 world→NED 平移、Yaw 偏置以及 Z 轴方向关系，避免在状态机各处混写坐标转换。

---

## 11. V1 后续升级接口

当前结构刻意保留以下升级空间：

1. `tree_hint` → 全局地图 / 全局目标搜索；
2. 固定观察弧线 → NBV / 主动感知；
3. `BranchModel.center` → root-backtracking 最优切点；
4. 固定 `cut_z_offset / tool_forward_offset` → 正式 cutting TCP 标定；
5. UAV 进刀 → Arm `movetcp` 主动进刀；
6. FAR/NEAR 固定距离 → 安全裕度 `S`；
7. 无碰撞规划 → MoveIt 2 / Planning Scene；
8. 固定 PREWORK → UAV + Arm 联合规划；
9. 仿真红枝识别 → 真实树枝语义识别与多目标选择。
