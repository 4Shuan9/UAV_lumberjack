# ros_bridge 操作说明

## 1. 目的

验证 ROS 2 可以通过 `ros_gz_bridge` 控制 Gazebo Harmonic 中的 3-DOF 机械臂关节。

控制链路：

```text
ROS 2
  ↓
std_msgs/msg/Float64
  ↓
ros_gz_bridge
  ↓
gz.msgs.Double
  ↓
Gazebo JointPositionController
  ↓
J1 / J2 / J3
```

## 2. 为什么需要 YAML 映射

Gazebo JointPositionController 默认生成的话题，例如：

```text
/model/lumberjack_arm/joint/j1/0/cmd_pos
```

该话题在 Gazebo Transport 中合法，但其中独立的 `/0/` token 不适合作为 ROS 2 topic 名。

因此使用 YAML 将 ROS 2 侧改成更简洁的名字：

```text
/lumberjack_arm/j1/cmd_pos
/lumberjack_arm/j2/cmd_pos
/lumberjack_arm/j3/cmd_pos
```

再分别映射到 Gazebo 原始话题。

## 3. bridge_arm.yaml

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
```

## 4. 启动 Gazebo

```bash
cd ~/UAV_lumberjack/sim/step2_3dof_arm
gz sim -v 4 -r arm_3dof_test.sdf
```

## 5. 启动 ros_gz_bridge

```bash
cd ~/UAV_lumberjack/sim/step3_ros_bridge
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
--ros-args \
-p config_file:=$(pwd)/bridge_arm.yaml
```

## 6. 检查 ROS 2 Topic

```bash
ros2 topic list | grep lumberjack
```

正常输出：

```text
/lumberjack_arm/j1/cmd_pos
/lumberjack_arm/j2/cmd_pos
/lumberjack_arm/j3/cmd_pos
```

检查接口：

```bash
ros2 topic info /lumberjack_arm/j1/cmd_pos
ros2 topic info /lumberjack_arm/j2/cmd_pos
ros2 topic info /lumberjack_arm/j3/cmd_pos
```

正常情况下每个 Topic 均为：

```text
Type: std_msgs/msg/Float64
Publisher count: 0
Subscription count: 1
```

其中 `Subscription count: 1` 对应 `ros_gz_bridge`。

## 7. ROS 2 控制测试

### J1 = +45°

```bash
ros2 topic pub --once /lumberjack_arm/j1/cmd_pos std_msgs/msg/Float64 "{data: 0.785398}"
```

### J2 = -30°

```bash
ros2 topic pub --once /lumberjack_arm/j2/cmd_pos std_msgs/msg/Float64 "{data: -0.523599}"
```

### J3 = -45°

```bash
ros2 topic pub --once /lumberjack_arm/j3/cmd_pos std_msgs/msg/Float64 "{data: -0.785398}"
```

### 回零

```bash
ros2 topic pub --once /lumberjack_arm/j1/cmd_pos std_msgs/msg/Float64 "{data: 0.0}"
ros2 topic pub --once /lumberjack_arm/j2/cmd_pos std_msgs/msg/Float64 "{data: 0.0}"
ros2 topic pub --once /lumberjack_arm/j3/cmd_pos std_msgs/msg/Float64 "{data: 0.0}"
```

## 8. 已验证结果

```text
[PASS] J1 ROS 2 → Gazebo 控制
[PASS] J2 ROS 2 → Gazebo 控制
[PASS] J3 ROS 2 → Gazebo 控制
[PASS] 三路 YAML topic 映射
[PASS] std_msgs/msg/Float64 → gz.msgs.Double 类型转换
```

至此，3-DOF 机械臂已经可以完全从 ROS 2 侧发送目标关节角。
