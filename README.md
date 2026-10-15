# UAV_lumberjack

<p align="center">
  <img src="/media/images/perception/cloud_point.jpg" width="100%">
</p>

<p align="center">
  <img src="/media/images/perception/agriculture_world.png" width="100%">
</p>

## System Requirements

| **Component** | **Version** |
| :--- | :--- |
| Ubuntu | 22.04.5 LTS |
| ROS 2 | Humble |
| Python | 3.10.12 |
| PX4 Autopilot | 1.16.2 |
| px4_msgs & px4_ros_com | release/1.16 |
| Micro-XRCE-DDS-Agent | 3.0.1 |
| Gazebo Harmonic | 8.15.0 |
| ros_gz_bridge | gzharmonic |
| QGroundControl | 5.0.8 |
| OpenCV | 4.13.0 |
| NumPy | 2.2.6 |
| SciPy | 1.15.3 |
| Open3D | 0.19.0 |

---

## Quick Start Guide
## Build

```bash
cd ~/UAV_lumberjack/ros2_ws

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

### Simulation

```bash
cd ~/UAV_lumberjack/ros2_ws

source /opt/ros/humble/setup.bash
source ~/ws_ros2/install/setup.bash
source install/setup.bash

ros2 launch uav_lumberjack_control uav_lumberjack_validation.launch.py
```

---

### Arm controller

```bash
cd ~/UAV_lumberjack/ros2_ws

source /opt/ros/humble/setup.bash
source ~/ws_ros2/install/setup.bash
source install/setup.bash

ros2 run uav_lumberjack_control arm_controller
```

---

### Env perception

```bash
cd ~/UAV_lumberjack/ros2_ws

source /opt/ros/humble/setup.bash
source ~/ws_ros2/install/setup.bash
source install/setup.bash

ros2 launch uav_lumberjack_perception perception.launch.py
```

---

### RViz

```bash
cd ~/UAV_lumberjack/ros2_ws

source /opt/ros/humble/setup.bash
source ~/ws_ros2/install/setup.bash
source install/setup.bash

rviz2 -d ~/UAV_lumberjack/sim/default.rviz \
  --ros-args -p use_sim_time:=true
```

---

### Offboard Demo

```bash
cd ~/UAV_lumberjack/ros2_ws

source /opt/ros/humble/setup.bash
source ~/ws_ros2/install/setup.bash
source install/setup.bash

ros2 run uav_lumberjack_control offboard_cutting_demo \
  --ros-args \
  --params-file \
  "$(ros2 pkg prefix --share uav_lumberjack_control)/config/offboard_cutting_demo.yaml"
```

## 最终枝条模型接口

```bash
ros2 topic echo /perception/branch_model --once
```

> 消息：`uav_lumberjack_interfaces/msg/BranchModel`

> 主要字段：`valid`, `center`, `direction`, `length`, `radius`, `fit_rms`, `point_count`

