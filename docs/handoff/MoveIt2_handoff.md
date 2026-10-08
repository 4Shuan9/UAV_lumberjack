# UAV–Arm 项目交接文档：MoveIt 2 移植与机械臂运动规划

## 1. 任务目标

本任务的目标不是重写现有 UAV–Arm 控制系统，而是在现有机械臂控制链路基础上，逐步引入 **MoveIt 2**，使机械臂具备更标准的：

- 机器人模型描述；
- 关节空间 / 笛卡尔空间目标规划；
- 逆运动学求解；
- 轨迹生成；
- 关节限位检查；
- 碰撞检测；
- Planning Scene 环境管理。

当前阶段先把 MoveIt 2 作为**机械臂运动规划层**接入，不直接负责 UAV 飞行控制。

整体思路：

```text
现有任务规划 / Offboard 状态机
        ↓
    ArmMotion Action
        ↓
现有 arm_controller
        ↓
  MoveIt 2 规划与求解
        ↓
机械臂 J2 / J3 / J4
```

尽量保留当前上层接口不变，让 MoveIt 2 替换或增强机械臂内部的“姿态求解 + 轨迹生成”部分。

---

# 2. 当前项目中与 MoveIt 2 相关的部分

项目根目录：

```bash
~/UAV_lumberjack
```

ROS 2 工作空间：

```bash
~/UAV_lumberjack/ros2_ws
```

主要控制包：

```text
uav_lumberjack_control
```

当前机械臂主要使用三个关节：

```text
J2
J3
J4
```

当前常用机械臂姿态：

```text
HOME
PREWORK
CUT_ALIGN
```

其中：

- **HOME**：起飞、飞行、返航、降落时使用；
- **PREWORK**：进入作业区前后的安全过渡姿态；
- **CUT_ALIGN**：根据目标枝条方向动态生成的切割姿态。

当前任务链大致为：

```text
HOME
→ FAR
→ PREWORK
→ CUT_ALIGN
→ NEAR
→ CUT
→ RETREAT_NEAR
→ RETREAT_FAR
→ PREWORK
→ HOME
```

机械臂动作目前通过：

```text
/lumberjack_arm/motion
```

进行调用。

现有 `ArmMotion.action` 中的主要模式包括：

```text
HOME
PREWORK
JOINT
TCP
```

因此建议第一阶段继续保留这个 Action 接口，让上层 Offboard 状态机仍然只需要发送“机械臂要去哪里”，MoveIt 2 在 `arm_controller` 内部完成规划。

---

# 3. 当前机械臂几何关系

当前机械臂主要关节为：

```text
J2 / J3 / J4
```

已知常用姿态：

```text
HOME
J2 = -30 deg
J3 = -150 deg
J4 = -90 deg
```

```text
PREWORK
J2 = -30 deg
J3 = -60 deg
J4 = 0 deg
```

当前临时 TCP 使用：

```text
chainsaw_body
```

需要注意：

> 当前真正参与切割的并不是 TCP 原点，而是链锯导板上的有效切割区域。

所以 MoveIt 2 初期可以先以 `chainsaw_body` 或一个明确的 tool link 作为末端参考，但后续最好增加专门的：

```text
cutting_tool_frame
```

或：

```text
cutting_point
```

用于描述链锯有效切割区域中的代表点。

当前链锯相关几何：

```text
chainsaw_body relative to wrist_link:
position = [0.060, 0.030, 0]
rotation ≈ Rx(90 deg)
```

链锯导板主要位于工具局部 `XY` 平面，因此导板法向主要对应工具局部：

```text
+Z
```

当前切割姿态的核心要求是：

\[
R_{\mathrm{world}}^{\mathrm{tool}}\mathbf e_z
\approx
\pm\mathbf d
\]

其中：

- \(\mathbf d\)：目标枝条主轴方向；
- \(\mathbf e_z=[0,0,1]^T\)：工具局部导板法向；
- CUT_ALIGN 的目标是让导板平面与枝条横截面一致。

---

# 4. 为什么要引入 MoveIt 2

目前项目已经能够通过自定义几何与运动学代码完成：

- HOME；
- PREWORK；
- CUT_ALIGN；
- TCP 目标；
- 基本 IK；
- 基本关节限位判断。

但是后续进入复杂树枝环境后，仅有自定义 IK 不够。

例如后续会遇到：

```text
目标姿态有多个 IK 解
↓
某些解虽然可达，但会碰树枝
↓
某些解虽然不碰撞，但接近关节极限
↓
某些解会使链锯或机械臂穿过树干
↓
需要在多个可行解中选更安全的解
```

所以 MoveIt 2 主要负责：

```text
IK
+
Joint Limit
+
Collision Checking
+
Planning Scene
+
Trajectory Planning
```

而 UAV 的位置和 yaw 仍然继续由 PX4 Offboard 管理。

---

# 5. 建议的总体架构

推荐不要让 MoveIt 2 直接接管整个 UAV–Arm 系统。

先采用下面的分层结构：

```text
                Task Planner
                     │
          ┌──────────┴──────────┐
          │                     │
        UAV                     Arm
          │                     │
     PX4 Offboard          MoveIt 2
          │                     │
 position / yaw        J2 / J3 / J4
```

即：

```text
PX4 Offboard
```

负责：

- UAV 位置；
- UAV yaw；
- FAR / NEAR / CONTACT；
- CUT_IN；
- RETREAT。

```text
MoveIt 2
```

负责：

- HOME；
- PREWORK；
- CUT_ALIGN；
- 机械臂 IK；
- 机械臂轨迹；
- 机械臂碰撞检测。

上层任务规划负责协调二者。

---

# 6. MoveIt 2 移植建议分阶段完成

不要一开始就直接做“复杂树枝 + 碰撞 + 动态 CUT_ALIGN”。

建议按下面顺序推进。

---

## Stage M1：建立 MoveIt 2 机器人模型

第一目标：

> 先让 MoveIt 2 正确识别机械臂 J2 / J3 / J4。

当前仿真机器人主要定义在：

```text
sim/models/x500_lumberjack/model.sdf
```

MoveIt 2 通常需要：

```text
URDF / Xacro
SRDF
```

所以首先需要为机械臂建立一份与当前 Gazebo 模型一致的机器人描述。

重点保证：

```text
joint name
joint axis
joint origin
joint limit
link transform
```

与仿真模型一致。

第一阶段只需要保留与机械臂有关的结构即可：

```text
base
↓
J2
↓
upper arm
↓
J3
↓
forearm
↓
J4
↓
wrist
↓
chainsaw/tool
```

不需要一开始就把无人机所有电机、桨叶、传感器都加入 MoveIt 2。

### M1 验收标准

在 RViz 中能够：

- 正确显示机械臂；
- 正确拖动 J2/J3/J4；
- 关节正方向与 Gazebo 一致；
- HOME 姿态一致；
- PREWORK 姿态一致。

---

# 7. Stage M2：配置 MoveIt Setup Assistant

机器人 URDF / Xacro 正确后，再使用 MoveIt Setup Assistant。

需要配置：

```text
Planning Group
End Effector
Joint Limits
Self Collision
Named States
```

建议 Planning Group：

```text
arm
```

包含：

```text
J2
J3
J4
```

建议先定义两个 Named State：

```text
HOME
PREWORK
```

这样后面可以直接调用：

```text
move_group -> HOME
```

或：

```text
move_group -> PREWORK
```

### 初期 End Effector

初期可以先把：

```text
chainsaw_body
```

作为工具末端。

后续再添加：

```text
cutting_tool_frame
```

用于描述实际有效切割区域。

### M2 验收标准

MoveIt RViz 中可以：

```text
Current State
→ Plan
→ Execute
```

完成：

```text
HOME → PREWORK
PREWORK → HOME
```

---

# 8. Stage M3：接通仿真机械臂控制

MoveIt 2 能规划之后，需要解决：

> MoveIt 2 生成的轨迹怎样真正驱动 Gazebo 中的 J2/J3/J4？

需要检查当前机械臂控制器的接口。

理想接口是：

```text
FollowJointTrajectory
```

即：

```text
trajectory_msgs/JointTrajectory
```

或：

```text
control_msgs/action/FollowJointTrajectory
```

如果当前 `arm_controller` 不是这种接口，不建议立刻删除现有控制器。

可以先做一个适配层：

```text
MoveIt 2 trajectory
        ↓
MoveItArmAdapter
        ↓
现有 arm_controller
```

等整个流程稳定后，再决定是否改为标准 `ros2_control`。

### M3 验收标准

MoveIt 2 在 RViz 点击：

```text
Plan & Execute
```

后，Gazebo 中机械臂真实运动。

并且：

```text
MoveIt Current State
```

能够同步 Gazebo 当前关节角。

---

# 9. Stage M4：保留现有 ArmMotion Action

现阶段非常建议保留：

```text
/lumberjack_arm/motion
```

不要让 Offboard 状态机直接调用 MoveIt 2。

建议修改后的内部结构：

```text
Offboard Cutting Demo
        ↓
ArmMotion Action
        ↓
arm_controller
        ↓
MoveIt 2
        ↓
Joint Trajectory
```

例如：

```text
ArmMotion HOME
```

内部转为：

```text
MoveIt Named Target: HOME
```

```text
ArmMotion PREWORK
```

内部转为：

```text
MoveIt Named Target: PREWORK
```

```text
ArmMotion JOINT
```

内部转为：

```text
MoveIt Joint Target
```

```text
ArmMotion TCP
```

内部转为：

```text
MoveIt Pose Target
```

这样现有 Offboard 状态机基本不需要修改。

---

# 10. Stage M5：把 CUT_ALIGN 接入 MoveIt 2

这是本任务最关键的一步。

Stage D 当前会根据目标枝条：

```text
BranchModel
```

得到：

```text
branch direction d
cutting plane
UAV yaw
CUT_ALIGN
```

当前 CUT_ALIGN 最核心约束：

\[
\mathbf n_{\mathrm{saw}}
\parallel
\mathbf d
\]

即：

```text
链锯导板平面
∥
枝条横截面
```

MoveIt 2 接入后，不建议只给一个固定：

```text
q2 / q3 / q4
```

而是逐步改为：

```text
目标 tool pose
        ↓
MoveIt IK
        ↓
多个候选解
        ↓
限位 / 碰撞检查
        ↓
选择可执行解
```

第一阶段可以只验证：

```text
当前 CUT_ALIGN pose
→ MoveIt IK
→ 是否能找到接近当前结果的关节解
```

之后再替换现有自定义 IK。

---

# 11. Stage M6：加入 Planning Scene

MoveIt 2 的真正价值主要在这里。

后续需要将环境中的：

```text
树干
树枝
输电线
```

加入：

```text
Planning Scene
```

初期不要直接加入复杂点云。

建议先使用简单几何体：

```text
Cylinder
Box
Sphere
```

例如：

```text
树干 → Cylinder
目标枝条 → Cylinder
其他枝条 → Cylinder
输电线 → Cylinder
```

Stage C 当前已经能够给出枝条：

```text
center
direction
length
radius
```

这非常适合直接转换成 MoveIt Collision Object 的 cylinder。

对应关系：

```text
BranchModel
    ↓
CollisionObject::CYLINDER
    ↓
Planning Scene
```

---

# 12. 目标枝条与碰撞模型要区别处理

这一点非常重要。

目标枝条既是：

```text
要切的对象
```

也是：

```text
碰撞物体
```

但链锯最终必须接触它。

所以不能简单设置：

```text
机械臂永远不能碰目标枝条
```

建议采用阶段化碰撞策略。

例如：

```text
FAR / PREWORK
目标枝条 = 禁止碰撞
```

```text
CUT_ALIGN / NEAR
链锯 guide bar 与目标枝条允许特定接触
其他机械臂 link 仍禁止碰撞
```

后续可以使用：

```text
Allowed Collision Matrix
```

仅允许：

```text
cutting_tool
↔
target_branch
```

而不允许：

```text
forearm
↔
target_branch
```

或：

```text
UAV body
↔
tree
```

---

# 13. MoveIt 2 与当前切割几何的关系

MoveIt 2 不负责决定：

> 应该切哪根树枝。

也不负责决定：

> 应该在哪里切。

这些仍然由上层任务规划决定。

完整关系建议保持：

```text
Stage C perception
        ↓
BranchModel
        ↓
Task / Cutting Geometry
        ↓
p_cut
cut plane
u_insert
CONTACT
FAR / NEAR
        ↓
      MoveIt 2
        ↓
机械臂可达性 / IK / 碰撞 / 轨迹
```

MoveIt 2 更像一个：

```text
“这个机械臂姿态能不能安全做到？”
```

的执行与验证模块。

---

# 14. 后续建议加入 Safety Margin

在 MoveIt 2 跑通后，不要只判断：

```text
IK exists = 可以执行
```

后续应进一步评价：

```text
距离关节极限还有多少
距离奇异位形有多远
机械臂离树干多远
链锯姿态是否合理
UAV 是否过度靠近树体
```

最终可以形成：

\[
S =
f(
S_{\mathrm{joint}},
S_{\mathrm{collision}},
S_{\mathrm{singularity}},
S_{\mathrm{clearance}},
S_{\mathrm{pose}}
)
\]

但这一部分属于后续优化，不是当前 MoveIt 2 移植的第一目标。

---

# 15. 推荐的软件分工

## MoveIt 2

负责：

```text
Planning Scene
Collision Checking
Motion Planning
Trajectory
Pose / Joint Goal
```

## Orocos KDL

后续可以用于：

```text
FK
Jacobian
基础 IK
奇异性分析
```

主要用于运动学分析和验证。

## pick_ik

暂时不需要立即接入。

当后面遇到：

```text
MoveIt 默认 IK 难以找到解
多解选择困难
姿态约束较强
```

时，再考虑 `pick_ik`。

---

# 16. 第一阶段不要做的事情

为了避免任务一下变得过大，初期先不要做：

```text
UAV + 机械臂全身联合规划
复杂树冠点云碰撞
实时动态障碍物
MoveIt Task Constructor
pick_ik
轨迹最优控制
复杂路径优化
```

第一目标只有：

```text
J2/J3/J4
↓
MoveIt 2
↓
HOME / PREWORK
↓
Gazebo 正确执行
```

跑通之后再继续。

---

# 17. 推荐实施顺序

建议严格按照以下顺序推进：

```text
M1
建立机械臂 URDF / Xacro
        ↓
M2
MoveIt Setup Assistant
        ↓
M3
MoveIt RViz 中 HOME / PREWORK 规划成功
        ↓
M4
MoveIt 轨迹驱动 Gazebo
        ↓
M5
接入现有 ArmMotion Action
        ↓
M6
接入动态 CUT_ALIGN
        ↓
M7
加入树枝 Planning Scene
        ↓
M8
加入 Collision Checking
        ↓
M9
再考虑更复杂的安全规划
```

---

# 18. 建议的目录结构

可以新增：

```text
ros2_ws/src/
├── uav_lumberjack_control/
├── uav_lumberjack_perception/
│
├── uav_lumberjack_description/
│   ├── urdf/
│   │   ├── arm.urdf.xacro
│   │   └── chainsaw.urdf.xacro
│   ├── meshes/
│   └── launch/
│
└── uav_lumberjack_moveit_config/
    ├── config/
    │   ├── uav_lumberjack.srdf
    │   ├── joint_limits.yaml
    │   ├── kinematics.yaml
    │   └── moveit_controllers.yaml
    └── launch/
```

其中：

```text
uav_lumberjack_description
```

负责机器人模型。

```text
uav_lumberjack_moveit_config
```

负责 MoveIt 2 配置。

原有：

```text
uav_lumberjack_control
```

继续负责：

```text
任务逻辑
机械臂 Action 接口
UAV–Arm 协调
```

---

# 19. 当前工作环境

常用 ROS 2 环境：

```bash
cd ~/UAV_lumberjack/ros2_ws

source /opt/ros/humble/setup.bash
source ~/ws_ros2/install/setup.bash
source install/setup.bash
```

当前主要环境：

```text
Ubuntu 22.04
ROS 2 Humble
Gazebo Harmonic
PX4 SITL
```

MoveIt 2 需要选择：

```text
ROS 2 Humble 对应版本
```

不要混用其他 ROS 2 发行版的教程或配置文件。

---

# 20. 第一阶段建议验收任务

第一阶段完成后，至少应能完成下面测试。

### Test 1：模型一致性

RViz 中：

```text
J2 / J3 / J4
```

方向和 Gazebo 一致。

### Test 2：HOME

MoveIt 2 能规划：

```text
Current State
→ HOME
```

并正确执行。

### Test 3：PREWORK

MoveIt 2 能规划：

```text
HOME
→ PREWORK
```

并正确执行。

### Test 4：返回 HOME

```text
PREWORK
→ HOME
```

过程中没有关节越界。

### Test 5：Pose Target

给定一个简单可达工具目标：

```text
tool pose
```

MoveIt 2 可以求出：

```text
J2 / J3 / J4
```

并执行。

---

# 21. 第二阶段验收任务

第一阶段稳定后再继续：

```text
BranchModel
↓
CUT_ALIGN target
↓
MoveIt IK
↓
Collision Check
↓
Execute
```

验证：

1. MoveIt 2 求出的导板法向是否满足：

\[
\left|
\mathbf n_{\mathrm{saw}}^T
\mathbf d
\right|
\approx 1
\]

2. 是否满足关节限位；

3. 是否与树干 / 非目标枝条发生碰撞；

4. 是否能够正常从 PREWORK 运动到 CUT_ALIGN。

---

# 22. 最终希望达到的接口

最终上层任务规划不需要知道 MoveIt 2 内部怎么求解。

例如 Offboard 只需要：

```text
send ARM_PREWORK
```

MoveIt 2 自动完成：

```text
当前机械臂状态
↓
规划
↓
碰撞检查
↓
执行
```

或者：

```text
send CUT_ALIGN(branch geometry)
```

内部完成：

```text
目标链锯姿态
↓
IK
↓
候选解
↓
碰撞检查
↓
轨迹规划
↓
执行
```

这样以后即使替换：

```text
IK solver
planner
collision model
```

上层任务状态机也不需要大改。

---

# 23. 建议优先完成的第一个小目标

不要直接从完整 CUT_ALIGN 开始。

第一个目标建议只做：

```text
机械臂模型
+
MoveIt 2
+
HOME
+
PREWORK
```

确认：

```text
MoveIt RViz
        ↓
Plan
        ↓
Execute
        ↓
Gazebo 中 J2/J3/J4 正确运动
```

这一条链完全跑通之后，再接入：

```text
ArmMotion Action
```

然后再做：

```text
CUT_ALIGN
```

最后才加入：

```text
Planning Scene
+
树枝碰撞模型
```

---

# 24. 交接后的第一步

建议先检查当前模型中的以下信息：

```text
J2 / J3 / J4 的：
- joint name
- parent link
- child link
- origin
- axis
- limit
```

然后建立最小机械臂 URDF / Xacro。

第一阶段暂时只需要实现：

```text
base
→ J2
→ J3
→ J4
→ chainsaw_body
```

并在 RViz 中验证其几何姿态与 Gazebo 一致。

确认模型一致之后，再进入 MoveIt Setup Assistant。

---

# 25. 一句话总结

当前 MoveIt 2 移植任务的核心不是重新设计整个 UAV–Arm 系统，而是：

\[
\boxed{
\text{保留现有任务状态机}
\rightarrow
\text{将 MoveIt 2 接入机械臂运动规划层}
\rightarrow
\text{逐步增加 IK、碰撞检测和安全规划}
}
\]

最先完成：

```text
HOME / PREWORK
```

然后完成：

```text
CUT_ALIGN
```

最后再接：

```text
Planning Scene + 树枝碰撞环境
```
