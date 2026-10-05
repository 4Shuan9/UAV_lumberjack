# UAV_lumberjack 机械臂运动学 V1

## 目的

本版本用于当前 X500 + 3DOF 前置机械臂的短期 FK/IK 验证和后续 Offboard 自动切割开发。
它是一个独立、可替换的手写运动学模块；后期采用 MoveIt 2 时可以整体替换，不要求与 MoveIt 2 内部求解器兼容。

## 输入角度

全部使用控制器已经采用的 **logical angle**：

- J2: -165° ~ +15°
- J3: -150° ~ +150°
- J4: -180° ~ +180°

不要把 Gazebo `/lumberjack_arm/joint_states` 的 raw rad 直接代入运动学。
当前关系仍为：`logical = raw + offset`，offset = `[-30°, -150°, -90°]`。

## 当前几何模型

参考坐标：`base_link`。

- base_link -> J2: `[0, 0, -0.110] m`
- J2 -> J3: `0.180 m`
- J3 -> J4: `0.170 m`
- J4 -> wrist_link origin: `0.025 m`
- wrist_link -> chainsaw_body: `[0.060, 0.030, 0] m`
- J2/J3 axis: `[0, -1, 0]`
- J4 axis: `[1, 0, 0]`

V1 暂定 `chainsaw_body` 原点为 TCP。以后若确定具体切割点，只需在该 FK 后继续乘一个固定 TCP 偏置，不需要重写 J2/J3/J4 运动学。

## FK

logical angle 为 `q2,q3,q4` 时：

- upper-arm pitch = `-q2`
- forearm pitch = `-(q2+q3)`
- wrist roll = `q4`
- chainsaw_body 相对 wrist 还有固定 `+90°` roll

因此 chainsaw_body 姿态为：

```text
R_B_TCP = Ry(-(q2+q3)) * Rx(q4 + 90°)
```

位置由 J2、J3、J4 到 wrist 以及 `[0.060,0.030,0]` 的锯体固定偏置逐级累加。

## IK

模块提供两种 IK：

1. `inversePosition()`：只给 TCP `[x,y,z]`，返回全部几何候选，并标记是否满足 logical joint limits。
2. `inversePose()`：给完整 TCP position + 机构允许的 pitch/roll 姿态，得到精确解。

因为机械臂只有 3DOF，不能实现任意 6D Pose。完整姿态必须满足：

```text
R = Ry(tool_pitch) * Rx(tool_roll)
```

其中 FK 对应：

```text
tool_pitch = -(q2+q3)
tool_roll  = q4 + 90°
```

## CLI 验证

构建后：

```bash
cd ~/UAV_lumberjack/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select uav_lumberjack_control --symlink-install
source install/setup.bash
```

自检：

```bash
ros2 run uav_lumberjack_control arm_kinematics_cli selftest
```

HOME FK：

```bash
ros2 run uav_lumberjack_control arm_kinematics_cli fk -30 -150 -90
```

PREWORK FK：

```bash
ros2 run uav_lumberjack_control arm_kinematics_cli fk -60 60 0
```

位置 IK 示例：

```bash
ros2 run uav_lumberjack_control arm_kinematics_cli ik 0.345 0.030 -0.265885
```

完整位姿 IK 示例（PREWORK 对应 tool pitch=0°, tool roll=90°）：

```bash
ros2 run uav_lumberjack_control arm_kinematics_cli ikpose 0.345 0.030 -0.265885 0 90
```

## 已验证基准

HOME `(-30,-150,-90)°` 的 FK 结果：

```text
TCP ≈ [-0.0991155, 0, -0.1700000] m
```

该结果与当前 SDF 在 raw joint=0 的 HOME link pose 链直接相乘得到的 `chainsaw_body` 原点一致。

另外通过程序自检验证：

- HOME FK -> pose IK -> HOME
- PREWORK FK -> pose IK -> PREWORK
- CUT_A `(-45,-45,0)` round trip
- CUT_B `(-30,-60,0)` round trip
- PREWORK position IK 包含原始关节解

## 当前边界

- 目前未做碰撞检测、trajectory planning、安全工作空间筛选。
- `inversePosition()` 只解决几何可达性，不能代替后期 MoveIt 2。
- TCP 暂定为 `chainsaw_body` 原点；正式切割规划前再定义切割区 TCP / primitive。
- 当前模块不会自动给机械臂下发动作，先用于数学和 Gazebo 对照验证。
