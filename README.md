# UAV_lumberjack

## 结构

```text
UAV_lumberjack/
├── docs/                         
├── media/                        
├── ros2_ws/src/
│   ├── uav_lumberjack_control/
│   ├── uav_lumberjack_interfaces/
│   └── uav_lumberjack_perception/
└── sim/                 
```

历史说明文档保留在 `docs/archive/legacy_steps/`。

## 环境

- Ubuntu 22.04
- ROS2 Humble
- PX4 1.16.2
- Gazebo Harmonic / gz-sim 8
- ros_gz_bridge
- Micro-XRCE-DDS-Agent

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
ros2 launch uav_lumberjack_control uav_lumberjack_sim.launch.py
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

消息：`uav_lumberjack_interfaces/msg/BranchModel`

主要字段：`valid`, `center`, `direction`, `length`, `radius`, `fit_rms`, `point_count`

