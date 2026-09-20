# UAV_lumberjack Step 9：固定前下视相机

## 1. 本阶段目标

在已经冻结的 UAV_lumberjack 2.0 机械平台基础上，为无人机增加一颗固定式前下视单目相机，并完成 Gazebo → ROS2 图像链路验证。

本阶段不加入 LiDAR、不加入树木目标、不修改机械臂控制逻辑。

---

## 2. 工程版本

开发目录：

```text
~/UAV_lumberjack/sim/step9_v2_camera
```

该目录由：

```text
step8_v2_platform
```

Launch：

```text
~/UAV_lumberjack/ros2_ws/src/uav_lumberjack_control/launch/uav_arm_v2.launch.py
```

当前已经切换到：

```text
step9_v2_camera
```

---

## 3. 相机模型

相机结构参考 PX4 官方模型：

```text
~/PX4-Autopilot/Tools/simulation/gz/models/mono_cam
~/PX4-Autopilot/Tools/simulation/gz/models/x500_mono_cam_down
```

当前采用固定相机：

```text
camera_link
sensor: imager
type: camera
```

图像参数：

```text
Resolution : 640 × 480
Frame rate : 30 Hz
HFOV       : 1.74 rad
```

初始安装采用前下视方式，随后手动微调：

```text
向前移动约 20 mm
向下移动约 5 mm
```

HOME 状态下机械臂不遮挡相机；PREWORK 状态下可以看到末端执行器。

---

## 4. Gazebo 图像话题

Gazebo 图像：

```text
/world/x500_lumberjack_world/model/x500_lumberjack/link/camera_link/sensor/imager/image
```

Gazebo CameraInfo：

```text
/world/x500_lumberjack_world/model/x500_lumberjack/link/camera_link/sensor/imager/camera_info
```

验证：

```bash
gz topic -l | grep -E "camera|image|imager"
```

---

## 5. ROS2 Bridge

Bridge 文件已改名为：

```text
~/UAV_lumberjack/sim/step9_v2_camera/bridge_gazebo_ros2.yaml
```

并已在 launch 中同步修改。

ROS2 输出话题：

```text
/camera/image_raw
/camera/camera_info
```

验证：

```bash
ros2 topic list | grep camera
```

实际测试相机帧率：

```text
约 30.2 Hz
```

与设定 30 Hz 一致。

CameraInfo 已成功发布：

```text
width  = 640
height = 480
distortion_model = plumb_bob
```

---

## 6. 图像显示

使用：

```bash
ros2 run rqt_image_view rqt_image_view
```

选择：

```text
/camera/image_raw
```

实际观察：

```text
HOME:
机械臂完全退出画面，视野干净。

PREWORK:
画面中可以看到末端执行器。

J2=0°, J3=0°:
存在局部自遮挡/贴近镜头现象，
属于极端调试姿态，暂不处理。
```

---

## 7. 当前相机链路

```text
Gazebo Camera
      ↓
gz.msgs.Image / CameraInfo
      ↓
ros_gz_bridge
      ↓
/camera/image_raw
/camera/camera_info
      ↓
ROS2 / OpenCV / 后续视觉算法
```

当前链路验证完成。

---

## 8. Step 9 结论

Step 9 已完成：

```text
固定前下视相机安装          ✅
Gazebo 图像输出           ✅
ROS2 自动桥接             ✅
CameraInfo               ✅
30 Hz 图像               ✅
rqt_image_view 实时显示   ✅
HOME/PREWORK 视野检查     ✅
```

因此可以进入下一阶段：

# Step 10：任务测试树

目标：

```text
主干
├── branch_1
├── target_branch
└── branch_3
```

第一版优先保证：

- 几何结构简单；
- 每根树枝实体独立；
- target_branch 可明确识别；
- 后续可以独立做 collision / detach；
- 相机和未来 MID360 都能围绕同一个目标调试。

暂时不追求真实树皮、树叶、风摆和木材断裂物理。
