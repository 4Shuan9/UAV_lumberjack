# Step 6 — J4 + 圆盘锯末端执行器

## 目标

在已完成的飞行 MVP 基础上增加：

- 缩短并轻量化 J2/J3 机械臂；
- J4 腕部滚转；
- 圆盘锯初始位于 J4 正上方；
- 圆盘锯独立连续旋转；
- 暂时只做 Gazebo 地面模型验证，不进入飞行测试。

## 相比 Step 5 的主要变化

### 机械臂缩短

```text
upper_arm: 0.22 m / 0.10 kg -> 0.18 m / 0.08 kg
forearm  : 0.16 m / 0.07 kg -> 0.12 m / 0.05 kg
wrist    : 0.03 kg          -> 0.025 kg
saw_disc : 0.04 kg          -> 0.025 kg
```

总机械臂质量约从 0.44 kg 降到 0.38 kg，同时明显减小末端力臂。

### J4

- J4 轴：X；
- J4 范围：±180°；
- J4 = 0° 时，圆盘位于 J4 正上方；
- 圆盘位置偏置：+Z 0.045 m。

### 圆盘锯

- 直径：0.10 m；
- 厚度：0.004 m；
- 旋转关节：`saw_spin_joint`；
- 连续旋转；
- 显式 Gazebo Topic：

```text
/lumberjack_arm/saw/cmd_vel
```

## 建立 Step 6

建议直接从已经稳定的 Step 5 复制：

```bash
cd ~/UAV_lumberjack/sim
cp -r step5_px4_flight step6_j4_saw
```

然后用本目录提供的 `model.sdf` 替换：

```text
step6_j4_saw/models/x500_lumberjack/model.sdf
```

如果 launch 中硬编码了 `step5_px4_flight`，将该路径改为：

```text
step6_j4_saw
```

## 第一轮只做 Gazebo 地面验证

先不要 ARM 飞机。

### 1. 检查锯片控制 Topic

```bash
gz topic -l | grep lumberjack_arm/saw
```

应看到：

```text
/lumberjack_arm/saw/cmd_vel
```

### 2. 启动圆盘

```bash
gz topic -t /lumberjack_arm/saw/cmd_vel -m gz.msgs.Double -p "data: 10.0"
```

### 3. 检查真实关节速度

```bash
gz topic -e -t /lumberjack_arm/joint_states
```

观察 `saw_spin_joint` 的 velocity 是否接近 `10 rad/s`。

### 4. 停止

```bash
gz topic -t /lumberjack_arm/saw/cmd_vel -m gz.msgs.Double -p "data: 0.0"
```

## 本轮验收

- HOME 时机械臂下垂明显减小；
- 圆盘初始位置位于 J4 正上方；
- 圆盘与 J4 间距更大；
- saw_spin_joint 能持续旋转；
- 不修改 J2/J3 已验证的控制增益，先观察短臂后的实际误差。
