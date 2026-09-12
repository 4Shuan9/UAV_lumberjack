# UAV_lumberjack 需求日志

## 1. 项目目标

项目名称：`UAV_lumberjack`

长期目标：构建“无人机 + 机械臂 + 树枝处理末端执行器”的空中作业系统，用于处理输电线路附近可能影响输电安全的树枝，目标树枝直径约 10 cm 以内。

当前阶段不追求完整树障处理功能，先验证“无人机 + 机械臂”组合在 PX4 + ROS 2 + Gazebo 中运行的可行性。

---

## 2. 当前开发环境

| 组件 | 版本 |
|---|---|
| Ubuntu | 22.04.5 LTS |
| ROS 2 | Humble |
| PX4 Autopilot | 1.16.2 |
| px4_msgs / px4_ros_com | release/1.16 |
| Micro-XRCE-DDS-Agent | 3.0.1 |
| Gazebo | Harmonic 8.12.0 |
| ros_gz_bridge | gzharmonic |

无人机平台优先采用 PX4 官方 `x500` 模型。

---

## 3. 本周阶段目标

完成一个可展示的最小 Demo：

1. PX4 X500 在 Gazebo 中正常起飞；
2. X500 稳定悬停；
3. 机腹刚性安装一个自制简化机械臂；
4. 第一版机械臂采用 3 DOF：
   - J1：Base Yaw
   - J2：Shoulder Pitch
   - J3：Elbow Pitch
5. ROS 2 侧可以输入“关节 + 目标角度”；
6. 对应机械臂关节能够运动；
7. 无人机悬停过程中机械臂仍可执行关节动作。

第一阶段评价优先级：

`能跑 > 易调试 > 外观像样 > 架构正规 > 动力学精确 > 功能丰富`

---

## 4. 当前明确暂不实现

本阶段暂不引入：

- MoveIt 2
- ros2_control
- 逆运动学 IK
- 末端 XYZ / Pose 控制
- UAV + Arm 联合规划
- 视觉识别
- 输电线 / 树枝场景
- 锯 / 剪末端执行器
- 接触与切削动力学
- 高精度质量与惯量参数
- 机械臂折叠与起降干涉处理

---

## 5. 当前机械臂方案

第一版先自行构建简化机械臂，而不是直接移植 UR5 / Panda 等成熟机械臂。

必须版本：

- 3 DOF
- J1 Base Yaw
- J2 Shoulder Pitch
- J3 Elbow Pitch

若进度顺利，再增加：

- J4 Wrist Pitch

机械臂刚性安装在 X500 机腹。

---

## 6. 控制与软件设计偏好

用户开发习惯：

- 传统控制 / ROS 2 控制节点：C++
- 视觉相关：Python

第一版机械臂控制希望采用：

- C++ / rclcpp
- 终端交互式输入
- 输入示例：`j1 30`、`j2 -20`、`home`
- 每次执行动作时进入 `BUSY`
- `BUSY` 状态下拒绝新的动作指令
- 当前动作完成后恢复 `IDLE`

无人机控制与机械臂控制在第一版中保持解耦。

---

## 7. 当前最短技术路线

第一版优先采用：

`ROS 2 C++ Node -> ros_gz_bridge -> Gazebo JointPositionController -> Gazebo Joint`

但在真正接入 ROS 2 之前，先单独验证 Gazebo 关节驱动功能。

当前调试原则：

1. 单独机械臂模型
2. 单关节 J1
3. 三关节 J1/J2/J3
4. ROS 2 Bridge
5. C++ Arm Controller
6. 集成 X500
7. 起飞悬停
8. 空中关节动作

---

## 8. 长期升级方向

后续可能逐步发展为：

`简化 3/4 DOF 机械臂`
→ `成熟开源机械臂模型`
→ `末端位置 / 姿态控制`
→ `UAV + Arm 联合规划与控制`
→ `树枝目标识别`
→ `锯 / 剪末端执行器`
→ `接触作业与抗扰控制`

