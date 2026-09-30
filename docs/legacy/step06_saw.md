# saw

## 本步骤完成内容

本步骤在原有 `X500 + 3DOF` 机械臂基础上完成了末端执行器扩展：

- 增加 J4（Wrist Roll）；
- 增加独立旋转圆锯；
- ROS2 可控制 J1/J2/J3/J4；
- ROS2 可控制锯片转速与停止；
- 增加 `home`、`status`、`test` 等控制命令；
- 关节运动加入梯形速度轨迹，减小空中动作时的瞬态扰动；
- 完成 `X500 + 4DOF Arm + Saw` 的起飞、悬停和空中机械臂动作验证。

当前演示使用：

```text
uav_arm_mvp_with_saw.launch.py
```

---

# 演示启动步骤

## 1. 编译工作空间

如果代码没有修改，可以跳过这一步。

```bash
cd ~/UAV_lumberjack/ros2_ws

source /opt/ros/humble/setup.bash

colcon build --packages-select uav_lumberjack_control

source install/setup.bash
```

---

## 2. 启动 Gazebo + PX4 + ROS-Gazebo Bridge

打开终端 1：

```bash
cd ~/UAV_lumberjack/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch uav_lumberjack_control uav_arm_mvp_with_saw.launch.py
```

该 launch 会自动：

```text
清理旧 Gazebo / PX4
        ↓
启动 Step6 Gazebo
        ↓
启动 ros_gz_bridge
        ↓
启动 PX4 SITL
```

正常情况下会看到：

```text
[LAUNCH] Cleaning old Gazebo / PX4 processes...
[LAUNCH] Cleanup complete.
```

之后 Gazebo 和 PX4 会自动启动。

---

## 3. 启动机械臂控制器

打开终端 2：

```bash
cd ~/UAV_lumberjack/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run uav_lumberjack_control arm_controller
```

进入控制器后可输入：

```text
status
home
j1 45
j2 -30
j3 45
j4 45
saw 500
saw off
test
help
quit
```

---

## 4. 推荐地面演示顺序

进入 `arm_controller` 后依次输入：

```text
status
```

```text
home
```

```text
j4 45
```

```text
j4 0
```

```text
saw 500
```

观察锯片旋转后停止：

```text
saw off
```

最后运行完整机械臂自动测试：

```text
test
```

`test` 会依次运动多个关节，最后自动回到 HOME。

---

## 5. 空中演示

保持终端 1 和终端 2 都在运行。

在 QGroundControl 中：

```text
ARM
→ 起飞
→ 悬停
```

悬停稳定后，在 `arm_controller` 中可执行：

```text
home
```

然后运行：

```text
test
```

用于演示机械臂在无人机悬停状态下进行完整动作。

如果需要演示锯片：

```text
saw 500
```

停止：

```text
saw off
```

动作结束后 PX4 会继续维持并恢复悬停。

---

## 6. 查看当前状态

任何时候都可以输入：

```text
status
```

查看：

```text
J1 / J2 / J3 / J4
命令角度
实际角度
位置误差
关节速度
锯片命令转速
锯片实际转速
```

---

## 7. 结束演示

先在机械臂控制器中关闭锯片并回零：

```text
saw off
```

```text
home
```

退出控制器：

```text
quit
```

最后在启动 Gazebo / PX4 的终端中按：

```text
Ctrl+C
```

即可结束本次演示。
