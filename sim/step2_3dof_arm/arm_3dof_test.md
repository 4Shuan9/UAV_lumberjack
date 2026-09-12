# arm_3dof_test.sdf 操作说明

## 1. 文件用途

`arm_3dof_test.sdf` 用于验证 `UAV_lumberjack` 第一版 3-DOF 简化机械臂在 Gazebo Harmonic 中的结构与关节位置控制。

当前机械臂包含：

- `base_link`：固定底座
- `yaw_link`：J1 上方旋转基座
- `upper_arm`：大臂
- `forearm`：小臂
- `j1`：Base Yaw
- `j2`：Shoulder Pitch
- `j3`：Elbow Pitch

外观优化：

- 大臂、小臂使用圆柱体
- J2、J3 关节处使用红色球形 visual 作为关节罩
- 红色关节罩不设置 collision，避免额外碰撞干扰

本阶段只验证 Gazebo 机械结构与关节控制，不涉及 ROS 2、PX4 和 X500。

---

## 2. 启动仿真

```bash
cd ~/UAV_lumberjack/sim/step2_3dof_arm
gz sim -v 4 -r arm_3dof_test.sdf
```

参数说明：

- `gz sim`：启动 Gazebo Sim
- `-v 4`：输出较详细的调试信息
- `-r`：启动后立即运行物理仿真
- `arm_3dof_test.sdf`：加载 3-DOF 机械臂测试世界

---

## 3. 检查三个关节控制 Topic

```bash
gz topic -l | grep lumberjack_arm
```

或：

```bash
gz topic -l | grep cmd_pos
```

正常情况下应出现：

```text
/model/lumberjack_arm/joint/j1/0/cmd_pos
/model/lumberjack_arm/joint/j2/0/cmd_pos
/model/lumberjack_arm/joint/j3/0/cmd_pos
```

说明 J1、J2、J3 的 `JointPositionController` 均已正常加载。

---

## 4. 关节控制测试

Gazebo 关节位置命令使用弧度 `rad`。

### J1：Base Yaw，转到 +45°

```bash
gz topic -t /model/lumberjack_arm/joint/j1/0/cmd_pos -m gz.msgs.Double -p 'data: 0.785398'
```

预期：

- `yaw_link`
- `upper_arm`
- `forearm`

作为 J1 下游机构一起绕 Z 轴旋转。

---

### J2：Shoulder Pitch，转到 -30°

```bash
gz topic -t /model/lumberjack_arm/joint/j2/0/cmd_pos -m gz.msgs.Double -p 'data: -0.523599'
```

预期：

- `upper_arm`
- `forearm`

一起绕 J2 的 Y 轴发生俯仰运动。

---

### J3：Elbow Pitch，转到 -45°

```bash
gz topic -t /model/lumberjack_arm/joint/j3/0/cmd_pos -m gz.msgs.Double -p 'data: -0.785398'
```

预期：

- `forearm` 绕 J3 发生肘部弯曲
- 上游 `base_link`、`yaw_link`、`upper_arm` 不因 J3 单独指令而改变自身关节状态

---

## 5. 当前组合姿态

依次执行：

```bash
gz topic -t /model/lumberjack_arm/joint/j1/0/cmd_pos -m gz.msgs.Double -p 'data: 0.785398'
```

```bash
gz topic -t /model/lumberjack_arm/joint/j2/0/cmd_pos -m gz.msgs.Double -p 'data: -0.523599'
```

```bash
gz topic -t /model/lumberjack_arm/joint/j3/0/cmd_pos -m gz.msgs.Double -p 'data: -0.785398'
```

对应关节空间状态约为：

```text
J1 = +45°
J2 = -30°
J3 = -45°
```

即：

```text
q = [0.785398, -0.523599, -0.785398] rad
```

---

## 6. 回零命令

### J1 回零

```bash
gz topic -t /model/lumberjack_arm/joint/j1/0/cmd_pos -m gz.msgs.Double -p 'data: 0.0'
```

### J2 回零

```bash
gz topic -t /model/lumberjack_arm/joint/j2/0/cmd_pos -m gz.msgs.Double -p 'data: 0.0'
```

### J3 回零

```bash
gz topic -t /model/lumberjack_arm/joint/j3/0/cmd_pos -m gz.msgs.Double -p 'data: 0.0'
```

---

## 7. 已验证结果

当前已经验证：

```text
J1 Base Yaw            PASS
J2 Shoulder Pitch      PASS
J3 Elbow Pitch         PASS
三关节组合姿态          PASS
圆柱形机械臂外观         PASS
J2/J3 红色关节罩         PASS
```

控制链路：

```text
Gazebo Transport
        ↓
gz.msgs.Double
        ↓
JointPositionController
        ↓
J1 / J2 / J3
        ↓
3-DOF 串联机械臂正常运动
```

下一阶段将验证：

```text
ROS 2
  ↓
ros_gz_bridge
  ↓
Gazebo JointPositionController
  ↓
3-DOF Arm
```
