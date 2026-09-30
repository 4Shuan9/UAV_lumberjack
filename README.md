# UAV_lumberjack

UAV + 机械臂 + 电锯 + RGB + MID360 的树枝感知与切割仿真工程。

## 当前结构

```text
UAV_lumberjack/
├── docs/                         # 阶段总结、历史文档、机械臂资料
├── media/                        # 图片与演示视频（视频仍保留在项目内）
├── ros2_ws/src/
│   ├── uav_lumberjack_control/
│   ├── uav_lumberjack_interfaces/
│   └── uav_lumberjack_perception/
└── sim/gazebo/                   # 当前唯一正式 Gazebo 仿真资源
```

历史 Step1~Step13 不再以重复工程目录保存；历史版本由 Git/GitHub 负责。历史说明文档保留在 `docs/archive/legacy_steps/`。

## 环境

- Ubuntu 22.04
- ROS2 Humble
- PX4 1.16.2
- Gazebo Harmonic / gz-sim 8
- ros_gz_bridge
- Micro-XRCE-DDS-Agent

> 图像链路不使用 `cv_bridge`；保持当前 NumPy / OpenCV 环境不变。

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

主要字段：`valid`, `center`, `direction`, `length`, `radius`, `fit_rms`, `point_count`。

## 说明

- `ros2_ws/build/`, `install/`, `log/` 属于可再生成内容，不包含在整理后的工程快照中。
- `.git/` 不包含在此 ZIP 中，因此 ZIP 是干净源码快照；原 GitHub 仓库继续承担版本历史管理。
- `media/videos/` 中的演示视频全部保留。
