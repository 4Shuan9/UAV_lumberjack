# arm_3dof_test.sdf 操作说明

## 1. 文件用途

`arm_3dof_test.sdf` 是 `UAV_lumberjack` 当前第一版 3-DOF 简化机械臂模型，用于独立验证机械结构、关节控制、PID 参数和真实关节状态反馈。

目录：

```text
~/UAV_lumberjack/sim/step2_3dof_arm
```

主要文件：

```text
arm_3dof_test.sdf
arm_3dof_test.md
arm_3dof_test.mp4
```

当前机械臂：

```text
base_link
    │
    └── J1 Base Yaw
           │
        yaw_link
           │
           └── J2 Shoulder Pitch
                    │
                 upper_arm
                    │
                    └── J3 Elbow Pitch
                             │
                          forearm
```

---

## 2. 当前关节定义

### J1 — Base Yaw

旋转轴：

```xml
<xyz>0 0 1</xyz>
```

范围：

```text
-180° ~ +180°
```

即：

```xml
<lower>-3.14159</lower>
<upper>3.14159</upper>
```

J1 负责整个机械臂绕竖直 Z 轴旋转。

---

### J2 — Shoulder Pitch

旋转轴：

```xml
<xyz>0 -1 0</xyz>
```

当前范围：

```text
-90° ~ +90°
```

J2 正方向已经调整为当前机械臂设计中较符合后续 UAV 安全工作空间的方向。

---

### J3 — Elbow Pitch

旋转轴：

```xml
<xyz>0 -1 0</xyz>
```

当前范围：

```text
-90° ~ +90°
```

J3 正方向与 J2 保持一致的机械臂弯曲语义。

---

## 3. 外观设计

当前模型已完成基础外观优化：

```text
upper_arm    圆柱形大臂
forearm      圆柱形小臂
J2           红色球形关节罩
J3           红色球形关节罩
```

红色关节罩：

```text
只有 visual
不添加 collision
```

避免装饰模型产生额外碰撞和物理抖动。

---

## 4. 当前 JointPositionController 参数

### J1

```xml
<p_gain>20.0</p_gain>
<i_gain>0.0</i_gain>
<d_gain>1.5</d_gain>

<cmd_max>25.0</cmd_max>
<cmd_min>-25.0</cmd_min>
```

J1 绕竖直轴运动，当前没有明显重力稳态负载，因此暂不使用积分项。

### J2

```xml
<p_gain>50.0</p_gain>
<i_gain>5.0</i_gain>
<d_gain>2.0</d_gain>

<i_max>5.0</i_max>
<i_min>-5.0</i_min>

<cmd_max>35.0</cmd_max>
<cmd_min>-35.0</cmd_min>
```

J2 承受大臂、小臂重力矩，因此加入积分项消除稳态误差。

### J3

```xml
<p_gain>35.0</p_gain>
<i_gain>3.0</i_gain>
<d_gain>1.5</d_gain>

<i_max>3.0</i_max>
<i_min>-3.0</i_min>

<cmd_max>30.0</cmd_max>
<cmd_min>-30.0</cmd_min>
```

当前参数已经满足独立机械臂 Demo 使用要求，暂时冻结，不继续针对独立模型过度调参。

---

## 5. Joint State Publisher

模型内已加入：

```xml
<plugin
  filename="gz-sim-joint-state-publisher-system"
  name="gz::sim::systems::JointStatePublisher">

  <topic>/lumberjack_arm/joint_states</topic>

  <joint_name>j1</joint_name>
  <joint_name>j2</joint_name>
  <joint_name>j3</joint_name>
</plugin>
```

Gazebo 输出：

```text
/lumberjack_arm/joint_states
```

消息类型：

```text
gz.msgs.Model
```

包含：

```text
J1 position / velocity
J2 position / velocity
J3 position / velocity
```

---

## 6. 启动机械臂仿真

```bash
cd ~/UAV_lumberjack/sim/step2_3dof_arm

gz sim -v 4 -r arm_3dof_test.sdf
```

---

## 7. 检查 Gazebo Topic

### 控制 Topic

```bash
gz topic -l | grep cmd_pos
```

正常：

```text
/model/lumberjack_arm/joint/j1/0/cmd_pos
/model/lumberjack_arm/joint/j2/0/cmd_pos
/model/lumberjack_arm/joint/j3/0/cmd_pos
```

### Joint State Topic

```bash
gz topic -l | grep joint_states
```

正常：

```text
/lumberjack_arm/joint_states
```

检查类型：

```bash
gz topic -i \
-t /lumberjack_arm/joint_states
```

应看到：

```text
gz.msgs.Model
```

---

## 8. Gazebo 直接控制测试

### J1 = +45°

```bash
gz topic \
-t /model/lumberjack_arm/joint/j1/0/cmd_pos \
-m gz.msgs.Double \
-p 'data: 0.785398'
```

### J2 = +45°

```bash
gz topic \
-t /model/lumberjack_arm/joint/j2/0/cmd_pos \
-m gz.msgs.Double \
-p 'data: 0.785398'
```

### J3 = +45°

```bash
gz topic \
-t /model/lumberjack_arm/joint/j3/0/cmd_pos \
-m gz.msgs.Double \
-p 'data: 0.785398'
```

当前关节空间：

```text
q = [45°, 45°, 45°]
```

---

## 9. 回零

```bash
gz topic -t /model/lumberjack_arm/joint/j1/0/cmd_pos -m gz.msgs.Double -p 'data: 0.0'

gz topic -t /model/lumberjack_arm/joint/j2/0/cmd_pos -m gz.msgs.Double -p 'data: 0.0'

gz topic -t /model/lumberjack_arm/joint/j3/0/cmd_pos -m gz.msgs.Double -p 'data: 0.0'
```

---

## 10. 查看真实 Joint State

```bash
gz topic -e \
-t /lumberjack_arm/joint_states
```

主要关注：

```text
joint {
  name: "j1"
  axis1 {
    position: ...
    velocity: ...
  }
}
```

以及 J2、J3。

其中：

```text
position    rad
velocity    rad/s
```

注意：

```text
joint.pose
```

描述的是关节坐标系几何位姿，不是实际关节角。

真实关节角应读取：

```text
axis1.position
```

---

## 11. 当前已验证结果

```text
[PASS] J1 Base Yaw
[PASS] J2 Shoulder Pitch
[PASS] J3 Elbow Pitch

[PASS] J1 ±180° 工作范围
[PASS] J2/J3 正方向重新定义
[PASS] 3-DOF 串联运动

[PASS] J1/J2/J3 JointPositionController
[PASS] J2/J3 PI/PD 重力稳态误差优化
[PASS] JointStatePublisher

[PASS] position feedback
[PASS] velocity feedback

[PASS] 圆柱形机械臂外观
[PASS] J2/J3 红色关节罩
```

当前独立机械臂模型已经满足 ROS 2 控制和下一阶段 X500 集成的基础要求。

---

## 12. 当前阶段注意事项

当前：

```text
HOME = [0°, 0°, 0°]
```

只是调试零位。

它不一定适合作为 UAV 起飞/降落姿态。

后续机械臂挂载到 X500 后，再根据：

```text
机体
桨盘
机械臂安装点
J1 方位
J2/J3 姿态
```

定义：

```text
FLIGHT_HOME
WORK_READY
安全工作空间
```

尤其需要考虑：

```text
J2 安全范围可能随 J1 角度变化
```

例如 J1 转向桨叶侧时，J2 的安全运动范围应进一步收紧。