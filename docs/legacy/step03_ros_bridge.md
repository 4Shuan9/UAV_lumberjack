# ROS 2 ↔ Gazebo Bridge 操作说明

## 1. 文件用途

本步骤使用 `ros_gz_bridge` 建立 ROS 2 与 Gazebo Harmonic 中 3-DOF 机械臂之间的通信。

目录：

```text
~/UAV_lumberjack/sim/step3_ros_bridge
```

当前主要文件：

```text
bridge_arm.yaml
ros_bridge.md
```

当前数据链路包含两个方向：

```text
控制：
ROS 2
  ↓
ros_gz_bridge
  ↓
Gazebo
  ↓
J1 / J2 / J3

反馈：
Gazebo
  ↓
JointStatePublisher
  ↓
ros_gz_bridge
  ↓
ROS 2 JointState
```

---

## 2. 为什么使用 YAML Bridge

Gazebo JointPositionController 默认控制 Topic：

```text
/model/lumberjack_arm/joint/j1/0/cmd_pos
```

其中 `/0/` 这一 token 在 Gazebo Transport 中可以正常使用，但不适合作为 ROS 2 topic token。

因此 ROS 2 侧使用：

```text
/lumberjack_arm/j1/cmd_pos
/lumberjack_arm/j2/cmd_pos
/lumberjack_arm/j3/cmd_pos
```

再通过 YAML 映射到 Gazebo 原始 Topic。

Joint State Topic 本身已经是：

```text
/lumberjack_arm/joint_states
```

因此 ROS 2 和 Gazebo 两侧可以使用相同名字。

---

## 3. 当前 bridge_arm.yaml

```yaml
- ros_topic_name: "/lumberjack_arm/j1/cmd_pos"
  gz_topic_name: "/model/lumberjack_arm/joint/j1/0/cmd_pos"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ

- ros_topic_name: "/lumberjack_arm/j2/cmd_pos"
  gz_topic_name: "/model/lumberjack_arm/joint/j2/0/cmd_pos"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ

- ros_topic_name: "/lumberjack_arm/j3/cmd_pos"
  gz_topic_name: "/model/lumberjack_arm/joint/j3/0/cmd_pos"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ

- ros_topic_name: "/lumberjack_arm/joint_states"
  gz_topic_name: "/lumberjack_arm/joint_states"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS
```

方向含义：

```text
ROS_TO_GZ
ROS 2 → Gazebo
用于发送目标关节角

GZ_TO_ROS
Gazebo → ROS 2
用于发送真实关节状态
```

---

## 4. 启动 Gazebo

终端 1：

```bash
cd ~/UAV_lumberjack/sim/step2_3dof_arm

gz sim -v 4 -r arm_3dof_test.sdf
```

---

## 5. 启动 Bridge

终端 2：

```bash
cd ~/UAV_lumberjack/sim/step3_ros_bridge

source /opt/ros/humble/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
--ros-args \
-p config_file:=$(pwd)/bridge_arm.yaml
```

保持终端运行。

---

## 6. 检查 ROS 2 Topic

```bash
ros2 topic list | grep lumberjack
```

正常应有：

```text
/lumberjack_arm/j1/cmd_pos
/lumberjack_arm/j2/cmd_pos
/lumberjack_arm/j3/cmd_pos
/lumberjack_arm/joint_states
```

---

## 7. 检查控制 Topic

例如 J1：

```bash
ros2 topic info /lumberjack_arm/j1/cmd_pos
```

正常：

```text
Type: std_msgs/msg/Float64
Publisher count: 0
Subscription count: 1
```

其中：

```text
Subscription count: 1
```

对应 `ros_gz_bridge`。

J2、J3 同理。

---

## 8. 检查 Joint State Topic

```bash
ros2 topic info /lumberjack_arm/joint_states
```

正常类型：

```text
sensor_msgs/msg/JointState
```

查看实时反馈：

```bash
ros2 topic echo /lumberjack_arm/joint_states
```

输出结构：

```yaml
name:
- j1
- j2
- j3

position:
- ...
- ...
- ...

velocity:
- ...
- ...
- ...

effort:
- ...
- ...
- ...
```

其中：

```text
position    rad
velocity    rad/s
```

`name[i]`、`position[i]`、`velocity[i]` 的相同索引属于同一关节。

程序中不要假定：

```text
position[0] 永远等于 J1
```

应根据：

```text
name[i]
```

匹配关节。

---

## 9. ROS 2 → Gazebo 控制测试

### J1 = +45°

```bash
ros2 topic pub --once \
/lumberjack_arm/j1/cmd_pos \
std_msgs/msg/Float64 \
"{data: 0.785398}"
```

### J2 = +45°

```bash
ros2 topic pub --once \
/lumberjack_arm/j2/cmd_pos \
std_msgs/msg/Float64 \
"{data: 0.785398}"
```

### J3 = +45°

```bash
ros2 topic pub --once \
/lumberjack_arm/j3/cmd_pos \
std_msgs/msg/Float64 \
"{data: 0.785398}"
```

### 回零

```bash
ros2 topic pub --once /lumberjack_arm/j1/cmd_pos std_msgs/msg/Float64 "{data: 0.0}"

ros2 topic pub --once /lumberjack_arm/j2/cmd_pos std_msgs/msg/Float64 "{data: 0.0}"

ros2 topic pub --once /lumberjack_arm/j3/cmd_pos std_msgs/msg/Float64 "{data: 0.0}"
```

---

## 10. 已验证结果

```text
[PASS] ROS 2 → Gazebo J1
[PASS] ROS 2 → Gazebo J2
[PASS] ROS 2 → Gazebo J3

[PASS] std_msgs/msg/Float64 → gz.msgs.Double

[PASS] Gazebo → ROS 2 JointState
[PASS] gz.msgs.Model → sensor_msgs/msg/JointState

[PASS] J1 actual position
[PASS] J2 actual position
[PASS] J3 actual position

[PASS] J1/J2/J3 actual velocity
```

最终通信结构：

```text
              q_cmd
ROS 2 ─────────────────► Gazebo
  ▲                         │
  │                         │
  └──────── q_actual ───────┘
```

因此机械臂已经具备 ROS 2 层面的真实状态反馈能力。