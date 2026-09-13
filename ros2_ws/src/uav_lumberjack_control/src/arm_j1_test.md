# arm_j1_test.cpp 操作说明

## 1. 文件用途

`arm_j1_test.cpp` 是 `UAV_lumberjack` ROS 2 C++ 控制链路的最小测试程序。

用途：

```text
验证 C++ / rclcpp
    ↓
std_msgs/msg/Float64
    ↓
ros_gz_bridge
    ↓
Gazebo JointPositionController
    ↓
J1
```

该程序目前保留作为：

```text
ROS 2 → Gazebo 最小 Smoke Test
```

完整 3-DOF 控制请使用：

```text
arm_controller.cpp
```

---

## 2. 文件位置

Workspace：

```text
~/UAV_lumberjack/ros2_ws
```

Package：

```text
uav_lumberjack_control
```

源码：

```text
~/UAV_lumberjack/ros2_ws/src/uav_lumberjack_control/src/arm_j1_test.cpp
```

说明：

```text
~/UAV_lumberjack/ros2_ws/src/uav_lumberjack_control/src/arm_j1_test.md
```

构建类型：

```text
ament_cmake
```

许可证：

```text
Apache-2.0
```

当前 package 使用：

```text
rclcpp
rclpy
std_msgs
sensor_msgs
```

其中 `arm_j1_test.cpp` 本身主要使用：

```text
rclcpp
std_msgs
```

---

## 3. 编译

```bash
cd ~/UAV_lumberjack/ros2_ws

source /opt/ros/humble/setup.bash

colcon build \
--packages-select uav_lumberjack_control \
--symlink-install

source install/setup.bash
```

检查 executable：

```bash
ros2 pkg executables uav_lumberjack_control
```

当前应看到：

```text
uav_lumberjack_control arm_j1_test
uav_lumberjack_control arm_controller
```

---

## 4. 启动 Gazebo

终端 1：

```bash
cd ~/UAV_lumberjack/sim/step2_3dof_arm

gz sim -v 4 -r arm_3dof_test.sdf
```

---

## 5. 启动 ros_gz_bridge

终端 2：

```bash
cd ~/UAV_lumberjack/sim/step3_ros_bridge

source /opt/ros/humble/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
--ros-args \
-p config_file:=$(pwd)/bridge_arm.yaml
```

---

## 6. 运行 J1 测试节点

终端 3：

```bash
cd ~/UAV_lumberjack/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run uav_lumberjack_control arm_j1_test
```

---

## 7. 正常动作

程序自动执行：

```text
等待 J1 bridge
     ↓
J1 → +45°
     ↓
保持约 2 s
     ↓
J1 → 0°
     ↓
节点退出
```

即：

```text
0°
 ↓
+45°
 ↓
0°
```

---

## 8. 已验证结果

```text
[PASS] ROS 2 Package
[PASS] ament_cmake
[PASS] C++ / rclcpp
[PASS] std_msgs/msg/Float64 Publisher
[PASS] ros_gz_bridge
[PASS] J1 0° → +45°
[PASS] J1 +45° → 0°
```

该程序的意义是证明：

```text
用户编写的 ROS 2 C++ 节点
        ↓
可以直接控制 Gazebo 机械臂
```

当前正式开发入口已经升级为：

```text
arm_controller.cpp
```

因此本程序后续主要用于快速排查 ROS 2 → Gazebo 控制链路。