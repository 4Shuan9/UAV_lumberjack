# Step11 相机参数与使用说明

## 1. 当前目标

Step11 用于给 UAV 前下视 RGB 相机增加 Gazebo 视场角可视化，并将仿真参数尽量贴近后期实物方案。

当前相机参数已暂时冻结。

---

## 2. 最终相机参数

```text
安装位置：
x = 0.135 m
y = 0
z = -0.025 m

安装俯角：
30°

分辨率：
1280 × 720

帧率：
30 Hz

水平视场角 HFOV：
约 90°

垂直视场角 VFOV：
约 58.7°

near：
0.1 m

far：
3000 m
```

说明：

- 相机向前安装并向下俯 30°。
- 1280×720 与后期 RK3588 + YOLO 实物方案更接近。
- YOLO 后期可将 720P 原图 resize / letterbox 到 640×640 等网络输入尺寸。
- 蓝色 FOV 视锥仅用于 Gazebo 可视化，不参与碰撞和控制。

---

## 3. 当前视野范围

垂直视场约为：

```text
58.7°
```

相机中心向下 30°，因此大致覆盖：

```text
上边界：约 -0.6°（接近水平线）
中心：   -30°
下边界：约 -59.4°
```

因此当前配置主要面向：

```text
前方
+
前下方
+
切割目标区域
```

对明显位于机体前上方的目标覆盖有限。

---

## 4. 启动 Step11

```bash
cd ~/UAV_lumberjack/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_lumberjack_control uav_arm_step11.launch.py
```

---

## 5. 查看相机画面

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

## 6. 启动机械臂控制器

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

---

## 7. 当前结论

Step11 相机参数暂时固定为：

```text
1280×720 @ 30 Hz
HFOV ≈ 90°
Pitch = 30°
x = 0.135 m
z = -0.025 m
```

当前版本优先保证前方和前下方作业区域感知。

下一步进入：

```text
Step12：MID360 类 3D LiDAR
```
