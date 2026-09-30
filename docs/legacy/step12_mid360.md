# Step12：MID360 类 3D LiDAR 仿真说明

## 1. 本阶段结论

Step12 已完成。

本阶段完成了 MID360 类 3D LiDAR 的外观、安装、Gazebo 点云、ROS2 桥接、TF 与 RViz2 显示。

当前已经具备后续 RGB + LiDAR 融合、目标树枝三维定位、自主接近与规划的基础。

---

## 2. 已完成内容

### MID360 外观与安装

- 使用 Livox MID360 官方 STEP 模型转换得到 Gazebo 可用模型。
- MID360 安装在相机上方。
- 保留当前人工调整后的安装位置与角度。
- MID360 本体保持正装，安装架提供约 20° 前下倾角。
- 航空插头方向已旋转到后方。
- 可见安装底板已去除。

### 3D LiDAR 仿真

使用 Gazebo Harmonic 原生 GPU LiDAR。

当前主要参数：

```text
水平视场角：360°
垂直视场角：-7° ~ +52°
刷新率：10 Hz
近距离：0.10 m
最大距离：40 m

水平采样：360
垂直采样：60
约 216000 points/s
```

说明：

真实 MID360 使用非重复扫描方式，本阶段采用规则 GPU LiDAR 网格进行工程近似。

---

## 3. ROS2 点云链路

当前点云链路：

```text
Gazebo gpu_lidar
        ↓
/mid360/scan/points
gz.msgs.PointCloudPacked
        ↓
ros_gz_bridge
        ↓
/mid360/points
sensor_msgs/msg/PointCloud2
```

已验证 ROS2 点云发布频率约：

```text
10 Hz
```

---

## 4. TF

当前 MID360 TF 关系：

```text
base_link
    ↓
mid360_mount_link
    ↓
mid360_link
    ↓
x500_lumberjack/mid360_link/mid360_gpu_lidar
```

静态 TF 已集成进：

```text
uav_arm_step12.launch.py
```

因此以后无需再手动启动 3 个 `static_transform_publisher`。

RViz2 当前可使用：

```text
Fixed Frame:
base_link
```

---

## 5. Camera FOV 与 LiDAR

Gazebo 中仍保留相机蓝色 FOV 可视化框。

但该 FOV 辅助框已经设置为对 MID360 不可见，因此：

```text
Gazebo 中能看到蓝色 FOV
MID360 点云不会扫描到蓝色 FOV
```

无人机机体、螺旋桨等真实结构目前仍会进入点云。

自身点云过滤暂时不做，留到后期点云处理算法阶段。

---

# 6. 常用启动命令

## 6.1 启动整个 Step12

```bash
cd ~/UAV_lumberjack/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch uav_lumberjack_control uav_arm_step12.launch.py
```

该 launch 当前负责启动：

```text
Gazebo
PX4 SITL
ros_gz_bridge
目标枝接触监测
自动切割逻辑
MID360 静态 TF
```

---

## 6.2 启动机械臂控制器

另开终端：

```bash
cd ~/UAV_lumberjack/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run uav_lumberjack_control arm_controller
```

常用命令：

```text
j2 <deg>
j3 <deg>
j4 <deg>

saw <rpm>
saw off

home
prework
ready
status
```

示例：

```text
j2 -45
j3 -45
j4 0

saw 1000
```

---

## 6.3 查看 RGB 相机

```bash
cd ~/UAV_lumberjack/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run rqt_image_view rqt_image_view
```

选择：

```text
/camera/image_raw
```

---

## 6.4 查看 MID360 点云

```bash
source /opt/ros/humble/setup.bash
source ~/UAV_lumberjack/ros2_ws/install/setup.bash

rviz2
```

RViz2 设置：

```text
Fixed Frame:
base_link

Add:
PointCloud2

Topic:
/mid360/points

Style:
Points

Color Transformer:
AxisColor
```

---

## 6.5 检查 Gazebo MID360 topic

```bash
gz topic -l | grep mid360
```

正常应看到：

```text
/mid360/scan
/mid360/scan/points
```

检查 Gazebo 点云类型：

```bash
gz topic -i -t /mid360/scan/points
```

应包含：

```text
gz.msgs.PointCloudPacked
```

---

## 6.6 检查 ROS2 点云

```bash
ros2 topic list | grep mid360
```

正常应看到：

```text
/mid360/points
```

检查类型：

```bash
ros2 topic info /mid360/points
```

应为：

```text
sensor_msgs/msg/PointCloud2
```

检查频率：

```bash
ros2 topic hz /mid360/points
```

正常约：

```text
10 Hz
```

检查 PointCloud2 frame：

```bash
ros2 topic echo /mid360/points --once --field header
```

当前 frame：

```text
x500_lumberjack/mid360_link/mid360_gpu_lidar
```

---

# 7. 当前暂不处理的内容

以下内容不是 Step12 平台搭建阶段必须完成的，因此暂时留到后期算法阶段：

```text
无人机自身点云过滤
螺旋桨点云过滤
FAST-LIO2
FAST-LIVO2
真实 Livox MID360 驱动
LiDAR + IMU 定位
RGB + LiDAR 融合
点云目标提取
```

---

# 8. Step12 完成后的状态

当前实验平台已经具备：

```text
PX4 + X500
3DOF 机械臂
链锯末端
目标枝接触与自动切断
RGB Camera
Camera FOV
MID360 类 3D LiDAR
ROS2 PointCloud2
TF
RViz2
Gazebo + PX4 + ROS2 完整链路
```

至此，仿真实验硬件与基础系统框架基本搭建完成。

下一阶段进入：

```text
Step13：RGB + LiDAR 融合基础
```

主要目标：

```text
确认 CameraInfo
确认 Camera / LiDAR 外参
MID360 点云变换到相机坐标系
将 LiDAR 点投影到 RGB 图像
为后续 YOLO + LiDAR 三维目标定位做准备
```
