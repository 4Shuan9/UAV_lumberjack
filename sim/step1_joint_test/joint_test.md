# joint_test.sdf 操作说明

## 1. 文件用途

`joint_test.sdf` 用于验证 Gazebo Harmonic 中最基础的单关节位置控制链路。

当前模型仅包含：

- `base_link`：固定底座
- `arm_link`：测试连杆
- `j1`：单自由度旋转关节（revolute）
- `JointPositionController`：Gazebo 内置关节位置控制器

本实验不涉及 ROS 2、PX4 和 X500。

---

## 2. 启动仿真

在当前目录执行：

```bash
gz sim -v 4 -r joint_test.sdf
```

参数说明：

- `gz sim`：启动 Gazebo Sim
- `-v 4`：输出较详细的调试日志
- `-r`：启动后立即运行物理仿真
- `joint_test.sdf`：加载当前 SDF 世界文件

---

## 3. 查看 J1 控制话题

```bash
gz topic -l | grep j1
```

正常情况下应看到：

```text
/model/joint_test/joint/j1/0/cmd_pos
```

说明 `JointPositionController` 已为 J1 创建目标位置控制话题。

---

## 4. 控制 J1

Gazebo 关节位置命令使用 **弧度 rad**。

### J1 转到 +45°

```bash
gz topic -t /model/joint_test/joint/j1/0/cmd_pos -m gz.msgs.Double -p 'data: 0.785398'
```

### J1 转到 -45°

```bash
gz topic -t /model/joint_test/joint/j1/0/cmd_pos -m gz.msgs.Double -p 'data: -0.785398'
```

### J1 回零

```bash
gz topic -t /model/joint_test/joint/j1/0/cmd_pos -m gz.msgs.Double -p 'data: 0.0'
```

其中：

- `-t`：指定 Gazebo Transport topic
- `-m`：指定消息类型，这里为 `gz.msgs.Double`
- `-p`：指定发送的数据内容
- `data`：目标关节位置，单位为 rad

常用角度换算：

```text
0°   = 0 rad
30°  ≈ 0.523599 rad
45°  ≈ 0.785398 rad
90°  ≈ 1.570796 rad
```

---

## 5. 已验证结果

当前实验已验证：

```text
Gazebo Transport
    ↓
gz.msgs.Double
    ↓
JointPositionController
    ↓
J1 revolute joint
    ↓
arm_link 正常旋转
```

测试结果：

- J1 +45°：正常
- J1 -45°：正常
- J1 回零：正常

因此 `joint_test.sdf` 可作为后续 3-DOF 机械臂开发的基础参考文件。
