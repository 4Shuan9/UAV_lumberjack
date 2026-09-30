# Step10 V2 Tree — 操作手册

## 1. Step10 目标

Step10 在前面 X500 + 3DOF 机械臂 + 电动链锯 + 相机的基础上，加入树木目标与自动切割逻辑。

当前已完成：

- `test_tree` 测试树模型
- 红色 `target_branch` 可脱落目标枝
- 链锯导板黄色环形 `cutting zone`
- Gazebo 接触检测
- ROS2 目标枝接触识别
- 锯速阈值判断
- 有效接触时间累计
- 满足条件后自动发送 detach，使红色树枝掉落

当前自动切割演示参数：

```text
Saw 阈值       = 800 rpm
有效接触时间   = 0.8 s
接触失联容忍   = 0.25 s
```

因此：

```text
500 rpm  + 接触树枝 → 不会切断
1000 rpm + 接触树枝约 0.8 s → 自动切断红色树枝
```

> 注：800 rpm 是当前仿真任务演示阈值，不代表真实链锯物理切割阈值。

---

## 2. 当前目录

Step10 仿真目录：

```bash
~/UAV_lumberjack/sim/step10_v2_tree
```

主要结构：

```text
step10_v2_tree/
├── bridge_gazebo_ros2.yaml
├── models/
│   ├── target_branch/
│   │   ├── model.config
│   │   └── model.sdf
│   ├── test_tree/
│   │   ├── model.config
│   │   └── model.sdf
│   └── x500_lumberjack/
│       ├── meshes/
│       │   └── cutting_zone_ring.stl
│       ├── model.config
│       └── model.sdf
└── worlds/
    └── x500_lumberjack_world.sdf
```

ROS2 控制包：

```bash
~/UAV_lumberjack/ros2_ws/src/uav_lumberjack_control
```

主要程序：

```text
arm_controller.cpp
target_contact_monitor.cpp
auto_cut_controller.cpp
uav_arm_v2.launch.py
```

---

## 3. 启动 Step10

### 终端 1：启动 Gazebo + PX4 + ROS-Gazebo Bridge + 自动切割节点

```bash
cd ~/UAV_lumberjack/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_lumberjack_control uav_arm_v2.launch.py
```

正常情况下会自动启动：

```text
Gazebo
PX4
ros_gz_bridge
target_contact_monitor
auto_cut_controller
```

---

## 4. 启动机械臂控制器

### 终端 2

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
init
help
quit
```

例如：

```text
j2 -45
j3 -45
j4 0
```

开启链锯：

```text
saw 500
```

或：

```text
saw 1000
```

关闭：

```text
saw off
```

---

## 5. 接触检测

黄色环形区域为链锯的 `cutting zone`。

ROS2 目标接触状态：

```bash
ros2 topic echo /lumberjack_arm/target_contact
```

未接触红色目标枝：

```text
data: false
```

黄色环接触红色目标枝：

```text
data: true
```

接触允许存在短暂抖动，自动切割控制器设置了约 `0.25 s` 的短暂失联容忍。

---

## 6. 自动切割测试

### 测试 A：500 rpm

机械臂控制器输入：

```text
saw 500
```

然后让黄色 `cutting zone` 接触红色目标枝。

预期：

```text
500 rpm < 800 rpm 阈值
→ 不累计有效切割
→ 红色树枝不会掉落
```

### 测试 B：1000 rpm

输入：

```text
saw 1000
```

让黄色 `cutting zone` 接触红色目标枝，并尽量保持约 0.8 s。

预期：

```text
Saw > 800 rpm
+
target_contact = true
+
有效接触累计 >= 0.8 s
↓
CUT SUCCESS
↓
自动发送 /target_branch/detach
↓
红色目标枝掉落
```

---

## 7. 手动测试树枝脱落

如需跳过自动切割逻辑，直接测试 DetachableJoint：

```bash
gz topic -t /target_branch/detach \
-m gz.msgs.Empty \
-p " "
```

预期：

```text
只有红色外侧目标枝掉落
木色内侧树枝仍固定在树上
```

---

## 8. 可选调试指令

查看 cutting zone 相关 Gazebo topic：

```bash
gz topic -l | grep cutting_zone
```

查看 ROS2 cutting zone / target contact topic：

```bash
ros2 topic list | grep -E 'cutting_zone|target_contact'
```

查看接触判断：

```bash
ros2 topic echo /lumberjack_arm/target_contact
```

查看切割进度：

```bash
ros2 topic echo /lumberjack_arm/cut_progress
```

---

## 9. 当前 Step10 工作流程

```text
QGC / PX4 控制无人机接近目标
        ↓
机械臂调整姿态
        ↓
启动链锯
        ↓
黄色 cutting zone 接触红色目标枝
        ↓
target_contact = true
        ↓
Saw > 800 rpm
        ↓
有效接触累计 0.8 s
        ↓
CUT SUCCESS
        ↓
DetachableJoint detach
        ↓
红色目标枝掉落
```

---

## 10. 当前验证结果

已验证：

```text
500 rpm  → 接触目标枝不会掉落
1000 rpm → 接触目标枝达到条件后自动掉落
```

因此 Step10 的“目标枝接触检测 + 锯速阈值 + 自动切割”功能已经跑通。

下一阶段：

```text
Step11：相机视场角可视化
Step12：MID360 类 3D LiDAR
```
