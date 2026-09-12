# arm_j1_test.cpp 操作说明

## 1. 文件用途

`arm_j1_test.cpp` 用于验证 C++ / ROS 2 节点可以通过 `ros_gz_bridge` 控制 Gazebo Harmonic 中的 J1 关节。

控制链路：

```text
arm_j1_test.cpp
      ↓
rclcpp Publisher
      ↓
/lumberjack_arm/j1/cmd_pos
      ↓
ros_gz_bridge
      ↓
Gazebo JointPositionController
      ↓
J1
```

## 2. ROS 2 Package

Package：

```text
uav_lumberjack_control
```

Workspace：

```text
~/UAV_lumberjack/ros2_ws
```

源码：

```text
~/UAV_lumberjack/ros2_ws/src/uav_lumberjack_control/src/arm_j1_test.cpp
```

构建类型：`ament_cmake`

许可证：`Apache-2.0`

依赖：

```text
rclcpp
rclpy
std_msgs
```

## 3. 编译

```bash
cd ~/UAV_lumberjack/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --packages-select uav_lumberjack_control --symlink-install

source install/setup.bash
```

检查 executable：

```bash
ros2 pkg executables uav_lumberjack_control
```

正常应看到：

```text
uav_lumberjack_control arm_j1_test
```

## 4. 启动 Gazebo

终端 1：

```bash
cd ~/UAV_lumberjack/sim/step2_3dof_arm
gz sim -v 4 -r arm_3dof_test.sdf
```

## 5. 启动 ros_gz_bridge

终端 2：

```bash
cd ~/UAV_lumberjack/sim/step3_ros_bridge
source /opt/ros/humble/setup.bash

ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$(pwd)/bridge_arm.yaml
```

## 6. 运行 C++ 测试节点

终端 3：

```bash
cd ~/UAV_lumberjack/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run uav_lumberjack_control arm_j1_test
```

## 7. 正常现象

节点启动后：

```text
Arm J1 test node started.
Bridge detected.
J1 command: 45.0 deg (0.785398 rad)
```

Gazebo 中：

```text
J1：0° → +45°
```

保持约 2 秒后：

```text
J1 command: 0.0 deg (0.000000 rad)
```

Gazebo 中：

```text
J1：+45° → 0°
```

最后：

```text
J1 test completed.
```

节点自动退出。

## 8. 已验证结果

```text
[PASS] ROS 2 package 创建
[PASS] Apache-2.0 License
[PASS] C++ / rclcpp 编译
[PASS] colcon build
[PASS] ROS 2 executable 注册
[PASS] C++ Publisher → ros_gz_bridge
[PASS] J1 0° → +45°
[PASS] J1 +45° → 0°
```

该测试证明机械臂已经可以由用户自行编写的 ROS 2 C++ 节点直接控制。
