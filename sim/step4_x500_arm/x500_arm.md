# x500_arm 操作说明

## 1. 目的

本阶段用于验证 PX4 X500 与 3-DOF 机械臂的静态机械集成，以及 ROS 2 对挂载后机械臂的控制。

当前验证链路：

```text
arm_controller.cpp
        ↓
ROS 2
        ↓
ros_gz_bridge
        ↓
Gazebo x500_lumberjack
        ↓
J1 / J2 / J3
```

本阶段 X500 仍通过 fixed joint 固定在 world 中，只验证机械臂挂载与控制，不进行飞行。

---

## 2. 相关文件

```text
~/UAV_lumberjack/sim/step4_x500_arm/
├── x500_arm_static.sdf
├── bridge_x500_arm.yaml
└── x500_arm.md
```

ROS 2 控制程序：

```text
~/UAV_lumberjack/ros2_ws/src/uav_lumberjack_control/src/arm_controller.cpp
```

---

## 3. 启动 Gazebo

终端 1：

```bash
cd ~/UAV_lumberjack/sim/step4_x500_arm
gz sim -v 4 -r x500_arm_static.sdf
```

当前模型名：

```text
x500_lumberjack
```

关节控制 Topic：

```text
/model/x500_lumberjack/joint/j1/0/cmd_pos
/model/x500_lumberjack/joint/j2/0/cmd_pos
/model/x500_lumberjack/joint/j3/0/cmd_pos
```

Joint State Topic：

```text
/lumberjack_arm/joint_states
```

---

## 4. 启动 ROS 2 Bridge

终端 2：

```bash
cd ~/UAV_lumberjack/sim/step4_x500_arm

ros2 run ros_gz_bridge parameter_bridge \
--ros-args \
-p config_file:=$(pwd)/bridge_x500_arm.yaml
```

检查：

```bash
ros2 topic list | grep lumberjack
```

应能看到：

```text
/lumberjack_arm/j1/cmd_pos
/lumberjack_arm/j2/cmd_pos
/lumberjack_arm/j3/cmd_pos
/lumberjack_arm/joint_states
```

---

## 5. 启动机械臂控制器

终端 3：

```bash
cd ~/UAV_lumberjack/ros2_ws
source install/setup.bash
ros2 run uav_lumberjack_control arm_controller
```

正常启动时应出现：

```text
All command bridges detected.
Joint state feedback detected.
```

---

## 6. 常用控制命令

```text
j1 <deg>     J1 Base Yaw
j2 <deg>     J2 Shoulder Pitch
j3 <deg>     J3 Elbow Pitch

home         三个关节回到 0°
status       查看关节状态
help         查看帮助
quit         退出
```

示例：

```text
j1 30
j2 -10
j3 60
status
home
```

当前软件关节范围：

```text
J1: -180° ~ +180°
J2:  -90° ~  +90°
J3:  -90° ~  +90°
```

---

## 7. 当前机械臂状态

当前 X500 挂载机械臂为轻量化版本，约 0.37 kg。

外观：

```text
arm_base   深灰
J1/yaw     黄色
J2大臂     蓝色
J3小臂     绿色
关节罩      红色
```

当前 J3 控制参数：

```xml
<p_gain>8.0</p_gain>
<i_gain>0.1</i_gain>
<d_gain>0.2</d_gain>

<i_max>0.1</i_max>
<i_min>-0.1</i_min>

<cmd_max>0.35</cmd_max>
<cmd_min>-0.35</cmd_min>
```

J1、J2、J3 均已验证可以通过 ROS 2 正常控制。

已验证示例：

```text
J1 = +30°
J2 = -10°
J3 = +60°
```

三个关节均可稳定到位，`home` 可正常回零。

---

## 8. 当前注意事项

### X500 仍被固定

`x500_arm_static.sdf` 中当前存在：

```xml
<joint name="x500_fixed_to_world" type="fixed">
  <parent>world</parent>
  <child>base_link</child>
</joint>
```

该 joint 仅用于静态机械调试。

正式进入 PX4 飞行模型后必须删除。

### J2 地面碰撞

当前零位机械臂横向伸展，在地面静态模型中，J2 较大的负角度可能使机械臂碰到地面。

因此当前静态调试建议暂时使用较小范围，例如：

```text
J2 >= -10° 左右
```

这只是当前地面调试限制，不是最终软件安全工作空间。

---

## 9. 当前阶段结论

已完成：

```text
X500 + 3DOF Arm 静态机械集成
        ↓
Gazebo 三关节控制
        ↓
ROS 2 Bridge
        ↓
C++ arm_controller
        ↓
JointState 闭环反馈
```

下一阶段：

```text
将 x500_lumberjack 制作为 PX4 可启动模型
        ↓
移除 world fixed joint
        ↓
PX4 SITL 启动
        ↓
地面检查
        ↓
起飞 / 悬停
```
