# joint_test.sdf 操作说明

## 1. 文件用途

`joint_test.sdf` 用于验证 Gazebo Harmonic 中最基础的单关节位置控制链路。

目录：

```text
~/UAV_lumberjack/sim/step1_joint_test
```

主要文件：

```text
joint_test.sdf
joint_test.md
```

模型包含：

```text
base_link
arm_link
j1                revolute joint
JointPositionController
```

本步骤只验证：

```text
Gazebo Transport
    ↓
gz.msgs.Double
    ↓
JointPositionController
    ↓
J1
```

不涉及 ROS 2、PX4 和 X500。

---

## 2. 启动仿真

```bash
cd ~/UAV_lumberjack/sim/step1_joint_test

gz sim -v 4 -r joint_test.sdf
```

其中：

```text
-v 4    输出较详细 Gazebo 日志
-r      启动后立即运行物理仿真
```

---

## 3. 检查 J1 控制 Topic

```bash
gz topic -l | grep j1
```

正常应出现：

```text
/model/joint_test/joint/j1/0/cmd_pos
```

说明 Gazebo `JointPositionController` 已正常加载。

查看详细信息：

```bash
gz topic -i \
-t /model/joint_test/joint/j1/0/cmd_pos
```

控制消息类型：

```text
gz.msgs.Double
```

---

## 4. J1 控制测试

Gazebo 关节位置命令单位为：

```text
rad
```

### +45°

```bash
gz topic \
-t /model/joint_test/joint/j1/0/cmd_pos \
-m gz.msgs.Double \
-p 'data: 0.785398'
```

### -45°

```bash
gz topic \
-t /model/joint_test/joint/j1/0/cmd_pos \
-m gz.msgs.Double \
-p 'data: -0.785398'
```

### 回零

```bash
gz topic \
-t /model/joint_test/joint/j1/0/cmd_pos \
-m gz.msgs.Double \
-p 'data: 0.0'
```

常用换算：

```text
0°    = 0 rad
30°   ≈ 0.523599 rad
45°   ≈ 0.785398 rad
90°   ≈ 1.570796 rad
180°  ≈ 3.141593 rad
```

Gazebo 命令参数：

```text
-t    指定 Topic
-m    指定消息类型
-p    指定消息内容
```

---

## 5. 已验证结果

```text
[PASS] Gazebo Harmonic 模型加载
[PASS] revolute joint
[PASS] JointPositionController
[PASS] Gazebo Transport Topic
[PASS] J1 +45°
[PASS] J1 -45°
[PASS] J1 回零
```

本步骤验证了最基础的：

```text
目标关节角
    ↓
Gazebo JointPositionController
    ↓
物理关节运动
```

该模型作为后续 3-DOF 机械臂控制的最小参考案例。