# arm_controller.cpp 操作说明

## 1. 文件用途

`arm_controller.cpp` 是当前 `UAV_lumberjack` 3-DOF 机械臂的主要 ROS 2 C++ 控制节点。

当前功能：

```text
J1 / J2 / J3 角度命令
真实 JointState 反馈
Commanded / Actual / Error / Velocity
软件关节限位
反馈式动作完成判断
BUSY / IDLE
BUSY 新动作拒绝
动作 Timeout
HOME
```

当前链路：

```text
                    q_cmd
arm_controller ─────────────► ros_gz_bridge
                                  │
                                  ▼
                         Gazebo Joint Controller
                                  │
                                  ▼
                              J1/J2/J3
                                  │
                                  │ q_actual
                                  ▼
                       JointStatePublisher
                                  │
                                  ▼
                           ros_gz_bridge
                                  │
                                  ▼
                         arm_controller
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
~/UAV_lumberjack/ros2_ws/src/uav_lumberjack_control/src/arm_controller.cpp
```

建议说明文件：

```text
~/UAV_lumberjack/ros2_ws/src/uav_lumberjack_control/src/arm_controller.md
```

---

## 3. 当前依赖

`package.xml`：

```xml
<depend>rclcpp</depend>
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
```

`arm_controller.cpp` 主要使用：

```text
rclcpp
std_msgs/msg/Float64
sensor_msgs/msg/JointState
```

---

## 4. 编译

```bash
cd ~/UAV_lumberjack/ros2_ws

source /opt/ros/humble/setup.bash

colcon build \
--packages-select uav_lumberjack_control \
--symlink-install

source install/setup.bash
```

检查：

```bash
ros2 pkg executables uav_lumberjack_control
```

正常：

```text
uav_lumberjack_control arm_controller
uav_lumberjack_control arm_j1_test
```

---

## 5. 启动顺序

### 终端 1：Gazebo

```bash
cd ~/UAV_lumberjack/sim/step2_3dof_arm

gz sim -v 4 -r arm_3dof_test.sdf
```

### 终端 2：ros_gz_bridge

```bash
cd ~/UAV_lumberjack/sim/step3_ros_bridge

source /opt/ros/humble/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
--ros-args \
-p config_file:=$(pwd)/bridge_arm.yaml
```

### 终端 3：arm_controller

```bash
cd ~/UAV_lumberjack/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run uav_lumberjack_control arm_controller
```

正常启动：

```text
UAV_lumberjack Arm Controller started.

Waiting for ros_gz_bridge command channels...
All command bridges detected.

Waiting for joint state feedback...
Joint state feedback detected.
```

---

## 6. 控制命令

### J1

```text
j1 <deg>
```

例如：

```text
j1 45
j1 -90
j1 135
```

范围：

```text
-180° ~ +180°
```

### J2

```text
j2 <deg>
```

例如：

```text
j2 45
j2 60
```

当前软件范围：

```text
-90° ~ +90°
```

### J3

```text
j3 <deg>
```

例如：

```text
j3 45
j3 60
```

当前软件范围：

```text
-90° ~ +90°
```

---

## 7. HOME

```text
home
```

当前定义：

```text
J1 = 0°
J2 = 0°
J3 = 0°
```

即：

```text
HOME = [0°, 0°, 0°]
```

注意：

```text
HOME 当前只是独立机械臂调试零位
```

不代表 UAV 起飞/降落时的最终安全姿态。

后续 X500 集成后将增加：

```text
FLIGHT_HOME
WORK_READY
```

---

## 8. Status

```text
status
```

输出：

```text
Joint   Commanded   Actual   Error   Velocity
```

含义：

```text
Commanded
最后发送的目标关节角

Actual
Gazebo JointState 实际关节角

Error
Commanded - Actual

Velocity
Gazebo 实际关节角速度
```

例如：

```text
Joint   Commanded     Actual       Error       Velocity
J1         45.00       45.00        0.00        0.00
J2         50.00       50.95       -0.95       -0.10
J3         54.00       54.50       -0.50       -0.04
```

单位：

```text
Position    deg
Velocity    deg/s
```

---

## 9. 动作完成判据

当前不是固定等待时间，而是使用真实反馈判断。

动作完成要求：

```text
|position error| <= 1.0°
```

并且：

```text
|velocity| <= 1.0°/s
```

连续满足：

```text
10 cycles
```

检查周期：

```text
50 ms
```

因此需要大约：

```text
10 × 50 ms = 0.5 s
```

连续稳定在允许范围内。

成功：

```text
[REACHED]
```

然后：

```text
Done. [IDLE]
```

---

## 10. Timeout

当前：

```text
Motion Timeout = 10 s
```

如果 10 秒内没有满足到位条件：

```text
[TIMEOUT]
```

会显示：

```text
Target
Actual
Error
Velocity
```

然后退出 BUSY：

```text
Motion ended with timeout/error. [IDLE]
```

Timeout 只是最大允许等待时间，并不是固定动作时间。

---

## 11. BUSY / IDLE

### IDLE

允许：

```text
j1
j2
j3
home
status
help
quit
```

### BUSY

允许：

```text
status
help
```

拒绝：

```text
j1
j2
j3
home
quit
```

例如 J2 正在运动：

```text
[BUSY] j2: 0.00 deg -> 50.00 deg
```

此时输入：

```text
j3 45
```

立即返回：

```text
[REJECTED] Arm is BUSY. Wait until the current motion finishes.
```

该命令不会缓存到动作结束后继续执行。

---

## 12. BUSY 时实时查看状态

动作过程中可以输入：

```text
status
```

例如：

```text
J2 Commanded = 50.00°
J2 Actual    = 51.43°
J2 Error     = -1.43°
J2 Velocity  = -0.15°/s

Arm State: BUSY
```

用于观察实时收敛过程。

---

## 13. 当前多线程结构

程序当前包含：

```text
ROS Spin Thread
    ↓
JointState callback

Terminal Thread
    ↓
读取用户命令

Motion Worker Thread
    ↓
执行当前机械臂动作
    ↓
等待真实反馈到位
```

因此可以做到：

```text
机械臂运动期间
仍然读取终端命令
```

运动命令会根据 BUSY 状态立即接受或拒绝。

---

## 14. 终端显示交错

由于：

```text
Terminal 输入
+
Motion Worker 后台输出
```

同时存在，可能出现：

```text
status[REACHED] ...
```

或者：

```text
[BUSY] > [BUSY] ...
```

这是简易终端界面的显示交错问题，不代表控制逻辑异常。

目前不作为重点处理。

后续采用：

```text
ROS Service
ROS Action
GUI
Joystick
```

等接口后，可避免这种纯终端交互显示问题。

---

## 15. 当前 PID 参数

### J1

```text
P = 20
I = 0
D = 1.5

cmd_max = +25
cmd_min = -25
```

### J2

```text
P = 50
I = 5
D = 2

i_max = +5
i_min = -5

cmd_max = +35
cmd_min = -35
```

### J3

```text
P = 35
I = 3
D = 1.5

i_max = +3
i_min = -3

cmd_max = +30
cmd_min = -30
```

当前 PID 参数已经满足 standalone 3-DOF Demo 使用需求。

暂时冻结。

待机械臂安装到 X500 后，由于：

```text
UAV 姿态
机体加速度
机械臂安装方向
质量分布
UAV-Arm 耦合
```

都会改变实际负载，因此再重新进行整体调参更合理。

---

## 16. 当前已验证功能

```text
[PASS] C++ / rclcpp 控制器
[PASS] J1/J2/J3 ROS 2 Command

[PASS] Gazebo JointState Feedback
[PASS] Actual Position
[PASS] Actual Velocity

[PASS] Commanded / Actual / Error / Velocity

[PASS] 软件 Joint Limit

[PASS] Position Completion Check
[PASS] Velocity Completion Check
[PASS] Stable Cycles
[PASS] Motion Timeout

[PASS] BUSY / IDLE
[PASS] BUSY 动作立即拒绝
[PASS] BUSY 时 status 可用

[PASS] HOME
```

---

## 17. 当前 standalone 控制链路

```text
Terminal Command
      ↓
arm_controller.cpp
      ↓
std_msgs/msg/Float64
      ↓
ros_gz_bridge
      ↓
Gazebo JointPositionController
      ↓
J1 / J2 / J3
      ↓
Gazebo JointStatePublisher
      ↓
gz.msgs.Model
      ↓
ros_gz_bridge
      ↓
sensor_msgs/msg/JointState
      ↓
arm_controller.cpp
      ↓
REACHED / TIMEOUT / BUSY / IDLE
```

---

## 18. 下一阶段

standalone 3-DOF 机械臂阶段已经基本完成。

下一步：

```text
X500
  +
3-DOF Arm
```

先只进行 Gazebo 机械集成：

```text
X500
  ↓
机腹 fixed joint
  ↓
lumberjack_arm
```

暂时不立即进行飞行。

首先验证：

```text
机械臂安装位置
桨叶干涉
J1 不同方向下的安全工作空间
J2/J3 安全角度
```

然后定义：

```text
HOME
FLIGHT_HOME
WORK_READY
```

最终再进入：

```text
PX4 起飞
    ↓
悬停
    ↓
机械臂空中动作
```