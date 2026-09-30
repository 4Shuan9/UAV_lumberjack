# Step 5 — PX4 Flight with 3-DOF Arm

## 1. 阶段目标

本阶段完成 `x500_lumberjack` 的首次完整飞行验证：

- PX4 SITL 接管自定义 X500 + 3DOF 机械臂模型
- QGC 解锁、起飞、悬停
- ROS2 通过 `ros_gz_bridge` 控制 J1 / J2 / J3
- 机械臂在空中进行大角度动作时，PX4 能保持飞行稳定
- 修复 J2 / J3 高频极限环
- 修复 PX4 `High Accelerometer Bias`
- 完成三关节飞行版控制参数定型

本阶段结论：**PASS**

---

## 2. 目录

```text
sim/step5_px4_flight/
├── models/
│   └── x500_lumberjack/
│       ├── model.config
│       └── model.sdf
├── worlds/
│   └── x500_lumberjack_world.sdf
└── px4_flight.md
```

机械臂 ROS2 bridge 继续复用：

```text
sim/step4_x500_arm/bridge_x500_arm.yaml
```

ROS2 控制器：

```text
ros2_ws/src/uav_lumberjack_control/src/arm_controller.cpp
```

---

## 3. 最终机械臂参数

### J1 — Base Yaw

```text
P = 10.0
I = 0.0
D = 0.6
cmd_max = +0.15
cmd_min = -0.15
```

J1 运动时会对机体产生轻微 yaw 反作用和小位移，但 PX4 能迅速补偿并恢复悬停。

### J2 — Shoulder Pitch

```text
P = 30.0
I = 0.0
D = 0.0
cmd_max = +0.50
cmd_min = -0.50
joint damping = 0.20
```

典型实测：

```text
+45 deg -> 44.57 deg
-45 deg -> -45.42 deg
最终速度约为 0
```

### J3 — Elbow Pitch

```text
P = 5.0
I = 0.0
D = 0.0
cmd_max = +0.15
cmd_min = -0.15
joint damping = 0.05
```

典型实测：

```text
+60 deg -> 59.39 deg
-60 deg -> -59.84 deg
最终速度约为 0
```

> J2 / J3 使用 `P + physical joint damping`，不再使用 PID 的 D 项做主要阻尼。

---

## 4. 启动顺序

### Terminal 1 — Gazebo

```bash
cd ~/UAV_lumberjack/sim/step5_px4_flight

gz sim -v 4 -r worlds/x500_lumberjack_world.sdf
```

等待 X500 落地稳定。

### Terminal 2 — PX4 SITL

```bash
cd ~/PX4-Autopilot

PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_NAME=x500_lumberjack \
./build/px4_sitl_default/bin/px4
```

正常情况下应看到：

```text
INFO [commander] Ready for takeoff!
```

QGC 使用 MAVLink 与 PX4 通信。

> 当前仅使用 QGC 控制飞行时，不需要启动 `MicroXRCEAgent`。
>
> 以后使用 ROS2 / `px4_msgs` 做 Offboard 控制时才需要 XRCE-DDS Agent。

### Terminal 3 — Arm bridge

```bash
cd ~/UAV_lumberjack/sim/step4_x500_arm

ros2 run ros_gz_bridge parameter_bridge \
--ros-args \
-p config_file:=$(pwd)/bridge_x500_arm.yaml
```

### Terminal 4 — Arm controller

```bash
cd ~/UAV_lumberjack/ros2_ws

ros2 run uav_lumberjack_control arm_controller
```

可用命令：

```text
j1 <deg>
j2 <deg>
j3 <deg>
home
status
help
quit
```

---

## 5. 已验证动作

地面和空中均完成多组测试，包括：

```text
J1: +45 / -45 / +60 / -60 / -75 deg
J2: +45 / -45 deg
J3: +60 / -60 deg
HOME
```

典型组合：

```text
J1 = -60 deg
J2 = -45 deg
J3 = -60 deg
```

机械臂最终能够稳定保持，关节速度接近 0，PX4 能维持飞行稳定。

---

## 6. 关键故障与处理

### 6.1 J2 / J3 高频极限环

原现象：

```text
J2 velocity ≈ ±0.54 rad/s
J3 velocity ≈ ±2.00 rad/s
```

速度几乎每个 2.5 ms physics step 正负翻转。

后果：

```text
机械臂高频振动
-> 机体振动
-> IMU / EKF 受到污染
-> 飞机严重漂移
```

处理方法：

- J2 / J3 的 PID D 项设为 0
- I 项设为 0
- 增加 SDF physical joint damping
- 大幅降低 controller `cmd_max / cmd_min`

修复后静止速度约为：

```text
10^-12 rad/s
```

可视为数值噪声级。

### 6.2 High Accelerometer Bias

异常参数曾出现：

```text
CAL_ACC0_XOFF = -0.4000
CAL_ACC0_YOFF = -0.3750
CAL_ACC0_ZOFF = +0.3678
```

导致：

```text
Preflight Fail: High Accelerometer Bias
QGC: Not Ready
```

恢复方法：

```bash
param reset CAL_ACC0_XOFF
param reset CAL_ACC0_YOFF
param reset CAL_ACC0_ZOFF
param reset CAL_ACC0_XSCALE
param reset CAL_ACC0_YSCALE
param reset CAL_ACC0_ZSCALE
param save
```

重启 PX4 后，EKF accel bias 恢复到接近 0，QGC 回到：

```text
Ready to Fly
```

---

## 7. 当前控制结论

当前参数已经满足本阶段目标，不再继续追求极小的单关节稳态误差。

对于飞行机械臂，评价重点应同时考虑：

```text
关节位置误差
关节速度
机械臂动作时间
UAV yaw / pitch / roll 扰动
UAV XY / Z 漂移
PX4 恢复速度
```

J1 大角度动作时的轻微 yaw 反作用属于 floating-base dynamic coupling。继续仅靠 PID 微调收益有限。

---

## 8. 下一阶段建议

下一阶段优先考虑机械臂上层轨迹整形，而不是继续磨 PID：

```text
位置阶跃
   ↓
梯形速度轨迹
   ↓
S 曲线速度 / 加速度轨迹
   ↓
减小机械臂反作用冲击
```

之后再进入：

```text
UAV-Arm 动态耦合分析
反作用力矩前馈
UAV-Arm 协同控制
工作姿态 / FLIGHT_HOME / WORK_READY 状态设计
末端执行器与切割扰动
```

---

## 9. 阶段状态

```text
Step 10.1  自定义 X500 + Arm 飞行模型       PASS
Step 10.2  PX4 attach existing model        PASS
Step 10.3  ARM / Motor chain                PASS
Step 10.4  带机械臂起飞与悬停               PASS
Step 10.5  悬停中控制机械臂                 PASS
Step 10.6  飞行版关节参数定型               PASS
```

**Step 5 / Step 10 大阶段完成。**
