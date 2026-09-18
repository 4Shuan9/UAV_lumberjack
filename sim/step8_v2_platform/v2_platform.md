# UAV_lumberjack V2.0 平台基线说明

## 1. 当前冻结版本

当前机械平台已经完成并通过手动验证，可作为后续传感器、感知和任务层开发的基线版本。

### Gazebo 模型
- 版本：V12 baked HOME
- 模型目录：
  `~/UAV_lumberjack/sim/step8_v2_platform/models/x500_lumberjack/model.sdf`

### ROS2 控制器
- 版本：V9 init
- 源文件：
  `~/UAV_lumberjack/ros2_ws/src/uav_lumberjack_control/src/arm_controller.cpp`

当前版本不要再直接覆盖旧 Step6 / Step7，后续修改继续在 `step8_v2_platform` 上进行。

---

## 2. 机械臂结构

当前机械臂取消 J1，由无人机 Yaw 负责水平朝向。

自由度：

- J2：Shoulder Pitch
- J3：Elbow Pitch
- J4：Wrist Roll
- Saw：末端旋转执行器

逻辑关节范围：

```text
J2: -165° ~ +15°
J3: -150° ~ +150°
J4: -180° ~ +180°
```

已知小问题：

```text
J2 = 0° 附近时，
J3 向正极限方向运动会与无人机机体发生干涉。
```

该问题暂不修改，后续归入“组合安全工作空间”处理，而不是简单缩小单关节机械极限。

---

## 3. 起飞 / 降落 HOME 姿态

Gazebo 模型已经把 HOME 姿态直接烘焙进初始几何，因此模型生成第一帧时机械臂已经处于折叠状态，不需要等待控制器运动到位。

```text
J2 = -30°
J3 = -150°
J4 = -90°
Saw = 0 rpm
```

控制器对 Gazebo 原始关节角和用户逻辑角进行了偏置映射，因此用户仍然使用上述逻辑角度进行控制。

命令：

```text
home
```

---

## 4. PREWORK 预工作姿态

```text
J2 = -60°
J3 = +60°
J4 = 0°
Saw = 0 rpm
```

命令：

```text
prework
```

或：

```text
ready
```

HOME / PREWORK 之间的运动仍使用梯形速度轨迹。

---

## 5. INIT 最大范围检查

原 `test` 已改为：

```text
init
```

执行流程：

```text
Saw OFF
    ↓
J2 / J3 / J4 -> 逻辑 0°
    ↓
J2: 0 → +15 → -165 → 0
    ↓
J3: 0 → +150 → -150 → 0
    ↓
J4: 0 → +180 → -180 → 0
    ↓
返回 HOME
```

J2/J3/J4 均使用原有梯形速度轨迹。

---

## 6. 当前梯形轨迹参数

```text
J2: vmax = 30 deg/s, amax = 25 deg/s²
J3: vmax = 45 deg/s, amax = 35 deg/s²
J4: vmax = 60 deg/s, amax = 40 deg/s²
Command period = 20 ms
```

短距离运动自动退化为三角速度轨迹。

Saw 当前速度命令本身仍为直接速度设定，不属于梯形速度规划。

---

## 7. 常用启动方式

### 终端 1：Gazebo + PX4

```bash
cd ~/UAV_lumberjack/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_lumberjack_control uav_arm_v2.launch.py
```

### 终端 2：机械臂控制器

```bash
cd ~/UAV_lumberjack/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run uav_lumberjack_control arm_controller
```

常用命令：

```text
home
prework
ready
init
status
j2 <deg>
j3 <deg>
j4 <deg>
saw <rpm>
saw off
quit
```

---

## 8. 下一阶段

下一阶段进入固定式视觉相机：

```text
Step 8.1
固定前下视相机
    ↓
Gazebo 图像验证
    ↓
ROS2 / OpenCV 图像接收
    ↓
确认 HOME / PREWORK 工作区视野
```

当前优先复用已经验证过的 PX4 / Gazebo 相机链路，不立即增加 LiDAR、树木或新的末端执行器。
