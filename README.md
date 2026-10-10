# UAV_lumberjack

<p align="center">
  <img src="/media/images/perception/agriculture_world.png" width="100%">
</p>

## System Requirements

| **Component** | **Version** |
| :--- | :--- |
| Ubuntu | 22.04.5 LTS |
| ROS 2 | Humble |
| PX4 Autopilot | 1.16.2 |
| px4_msgs & px4_ros_com | release/1.16 |
| Micro-XRCE-DDS-Agent | 3.0.1 |
| Gazebo Harmonic | 8.x |
| ros_gz_bridge | gzharmonic |

## 构建

```bash
cd ~/UAV_lumberjack/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 启动仿真

```bash
cd ~/UAV_lumberjack/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_lumberjack_control uav_lumberjack_validation.launch.py
```

## 启动感知

```bash
cd ~/UAV_lumberjack/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_lumberjack_perception perception.launch.py
```

## 最终枝条模型接口

```bash
ros2 topic echo /perception/branch_model --once
```

> 消息：`uav_lumberjack_interfaces/msg/BranchModel`

> 主要字段：`valid`, `center`, `direction`, `length`, `radius`, `fit_rms`, `point_count`

