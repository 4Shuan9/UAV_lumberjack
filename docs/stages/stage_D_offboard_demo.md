# 阶段 D：基于枝条几何模型的 UAV–Arm Offboard 自主切割闭环

> **阶段 A：** 验证 UAV–Arm 基础飞行与机械臂动作可行性。  
> **阶段 B：** 建立面向伐枝任务的 UAV–Arm、链锯、树枝与传感器仿真平台。  
> **阶段 C：** 基于 RGB–LiDAR 完成目标枝条三维感知与几何建模。  
> **阶段 D：** 将阶段 C 输出的枝条几何模型接入 UAV–Arm 作业控制，先完成一个可自动运行、可分段调试、可恢复的最小切割闭环。

Stage D 表面上是一个 Offboard 自动切割 Demo，但实际主要探索五个问题：

1. **根据枝条方向构造合理的切割平面**
2. **初步确定 UAV yaw 与机械臂姿态的联合关系**
3. **建立链锯有效切割区域与目标枝条之间的几何关系**
4. **确定 UAV 接近目标的方向及 FAR / NEAR / CUT_IN 作业轨迹**
5. **搭建可分段执行、恢复、一键执行的作业架构**

本阶段先验证：

\[
\boxed{
\text{BranchModel}
\rightarrow
\text{Cutting Geometry}
\rightarrow
\text{UAV--Arm Execution}
}
\]

这条最小闭环能够完整运行。

---

# 1. Stage D 的输入与总体流程

阶段 C 输出枝条模型：

\[
\mathcal B=
\left\{
\mathbf p_0,\mathbf d,L,r
\right\}
\]

其中：

- \(\mathbf p_0\)：枝条几何中心；
- \(\mathbf d\)：枝条单位主轴方向；
- \(L\)：枝条有效长度；
- \(r\)：枝条半径。

任务主流程为：

```text
TAKEOFF
→ OBSERVE / ARC_SCAN
→ TARGET_LOCK
→ FAR
→ PREWORK
→ CUT_ALIGN
→ NEAR
→ CUT
→ RETREAT
→ HOME
→ RETURN_HOME
→ LANDING
```

---

# 2. 问题一：根据枝条方向构造切割平面

阶段 C 已经获得枝条的单位主轴方向：

\[
\mathbf d
\]

若想横向切断树枝，最直观的方法是让链锯所在的**切割平面与枝条主轴垂直**。

容易想到：**根据枝条主轴 \(\mathbf d\) 寻找其垂直的向量。**

> 但与枝条主轴垂直的向量有**无数个**，且这些向量都位于枝条的横截面内，可理解为不同的候选切入方向。

因此，只找一个与枝条垂直的向量，难以直接确定链锯的切割平面应该怎么摆。

换个思路：**先确定切割平面。**

> 对于近似圆柱形的枝条，如果希望沿枝条横截面进行切割，那么切割平面一定与枝条主轴垂直，即切割平面的法向量平行于枝条主轴。

也就是：

\[
\boxed{
\text{切割平面}
\perp
\text{枝条主轴}
}
\]

等价于：

\[
\boxed{
\text{切割平面法向量}
\parallel
\text{枝条主轴}
}
\]

因此：

\[
\boxed{
\mathbf n_{\mathrm{cut}}
=
\pm\mathbf d
}
\]

其中：

- \(\mathbf n_{\mathrm{cut}}\)：切割平面的法向量；
- \(\pm\)：同一平面的两个相反法向方向。

若切割点为 \(\mathbf p_{\mathrm{cut}}\)，则切割平面为：

\[
\boxed{
\mathbf d^T
\left(
\mathbf p-\mathbf p_{\mathrm{cut}}
\right)
=
0
}
\]

其中 \(\mathbf p\) 表示平面上的任意一点,它们的点积为 0（即垂直）。

<p align="center">
  <img src="/media/images/cutting/cutting_plane.png" width="100%">
</p>

这样首先确定了“**链锯应该落在哪个切割平面上**”。下一步再考虑如何通过 UAV 与机械臂，把链锯导板真正摆到这个平面。

---

# 3. 问题二：初步确定 UAV yaw 与机械臂姿态的联合关系

切割平面确定后，下一步要解决：

> **怎样让链锯导板真正转到这个切割平面上？**

当前链锯导板主要位于 `chainsaw_body` 的局部 \(XY\) 平面，其局部法向为：

\[
\mathbf e_z=
\begin{bmatrix}
0\\
0\\
1
\end{bmatrix}
\]

若工具在 world 中的旋转矩阵为 \(R_{\mathrm{world}}^{\mathrm{tool}}\)，则导板法向为：

\[
\boxed{
\mathbf n_{\mathrm{saw}}
=
R_{\mathrm{world}}^{\mathrm{tool}}
\mathbf e_z
}
\]

前文已确定切割平面的法向为 \(\pm\mathbf d\)，因此希望：

\[
\boxed{
\mathbf n_{\mathrm{saw}}
\parallel
\mathbf d
}
\]

> 即链锯导板平面与枝条主轴垂直

为了评价对齐程度，定义：

\[
\boxed{
\eta_{\mathrm{align}}
=
\left|
\mathbf n_{\mathrm{saw}}^T
\mathbf d
\right|
}
\]

> 两个平行的非零单位向量的点积的绝对值为 1

当 \(\eta_{\mathrm{align}}\) 越接近 1，说明链锯导板与目标切割平面越一致。

## 仅调整 UAV yaw 无法完成三维切割姿态对齐

UAV yaw 只能改变无人机在**水平面内**的朝向。

<p align="center">
  <img src="/media/images/cutting/cutting_plane_n_saw.png" width="75%">
</p>

**枝条在三维空间中向上或向下斜着生长**，仅靠 yaw 无法让链锯导板平面与枝条横截面完全对齐，因此需要：

- **UAV yaw**：调整水平朝向；
- **J2/J3**：调整链锯位置和整体俯仰；
- **J4**：调整导板滚转方向。

因此最终由：

\[
\boxed{
\text{UAV yaw}
+
\text{J2 / J3 / J4}
\rightarrow
\text{CUT\_ALIGN}
}
\]

共同确定切割姿态。

机械臂末端姿态关系为：

\[
\boxed{
R_{\mathrm{world}}^{\mathrm{tool}}
=
R_z(\psi_{\mathrm{UAV}})
R_y(-(q_2+q_3))
R_x\left(q_4+\frac{\pi}{2}\right)
}
\]

所以程序需找到合适的：

\[
\left(
\psi_{\mathrm{UAV}},q_2,q_3,q_4
\right)
\]

使导板法向**尽量**满足（同时满足关节限位）：

\[
R_{\mathrm{world}}^{\mathrm{tool}}
\mathbf e_z
\approx
\pm\mathbf d
\]

> 实际代码在确定最终 CUT_ALIGN 时，还会结合后面计算的切入方向，用它进一步固定切割平面内剩余的姿态自由度。

---

# 4. 问题三：建立链锯有效切割区域与目标枝条的几何关系

> 不能直接将机械臂 TCP 作为实际切割点，若强行要求 TCP 与枝条切割点重合，可能导致链锯实际接触位置不合理，并产生较大的接触力矩和 UAV 姿态扰动。

<p align="center">
  <img src="/media/images/cutting/cutting_point.png" width="75%">
</p>

真正参与锯切的是链锯导板上的有效切割区域，链锯有效切割区域中的代表点在 UAV base 坐标系中的位置为：

\[
\mathbf p_{\mathrm{cutting,base}}
\]

目标枝条切割点在 world 坐标系中的位置为：

\[
\mathbf p_{\mathrm{branch}}
\]

则发生接触时 UAV base 的目标位置为：

\[
\boxed{
\mathbf p_{\mathrm{contact}}
=
\mathbf p_{\mathrm{branch}}
-
R_{\mathrm{world}}^{\mathrm{base}}
\mathbf p_{\mathrm{cutting,base}}
}
\]

CONTACT 不是提前写死的，而是由：

\[
\boxed{
\text{枝条几何}
+
\text{机械臂构型}
+
\text{链锯几何}
}
\]

共同决定。

> **先确定链锯有效切割区域应该落在哪里，再经过坐标系转换，反推出 UAV base 应该停在哪里**

这一步完成后，我们已经知道最终“**要切到哪里**”以及“**无人机最终接触位置在哪里**”，接下来再决定 UAV 应该从哪个方向接近这一位置。

---

# 5. 问题四：确定 UAV 接近目标的方向及 FAR / NEAR / CUT_IN 作业轨迹

CONTACT 确定后，下一步要解决：

> **无人机应该从哪个方向靠近 CONTACT，并怎样逐级进入切割区域？**

假设 UAV 原本朝 CONTACT 的沿着最短欧式距离的运动方向是：

\[
\mathbf t
\]

如果直接让 UAV 朝目标飞，运动方向中可能带有沿枝条主轴的分量，不一定是在横向切入，它可以分解成两部分：

\[
\boxed{
\mathbf t
=
\mathbf t_{\perp}
+
\mathbf t_{\parallel}
}
\]

<p align="center">
  <img src="/media/images/cutting/cutting_t.png" width="75%">
</p>

希望切入方向一定位于前面确定的切割平面内（\(\mathbf d\)  是单位向量）：

\[
\boxed{
\mathbf u_{\mathrm{insert}}
\perp
\mathbf d
}
\]

去掉 \(\mathbf t\) 中沿枝条主轴的分量得：

\[
\boxed{
\mathbf t_{\perp}
=
\mathbf t
-
\mathbf t_{\parallel}
}
\]

其中：

\[
\boxed{
\mathbf t_{\parallel}
=
\mathbf d
\left(
\mathbf t^T\mathbf d
\right)
}
\]

再归一化得到：

\[
\boxed{
\mathbf u_{\mathrm{insert}}
=
\frac{\mathbf t_{\perp}}
{\|\mathbf t_{\perp}\|}
}
\]

可以理解为：

> UAV 原本朝向树枝的方向里，可能混有“顺着树枝”的分量，去除这一部分，只保留横向分量，得位于枝条横截面内的切入方向。

有了 CONTACT 和切入方向后，再沿 \(\mathbf u_{\mathrm{insert}}\) 向后设置 NEAR 与 FAR两个工作位置：

\[
\boxed{
\mathbf p_{\mathrm{near}}
=
\mathbf p_{\mathrm{contact}}
-
d_{\mathrm{near}}
\mathbf u_{\mathrm{insert}}
}
\]

\[
\boxed{
\mathbf p_{\mathrm{far}}
=
\mathbf p_{\mathrm{contact}}
-
d_{\mathrm{far}}
\mathbf u_{\mathrm{insert}}
}
\]

其中：

\[
d_{\mathrm{far}}
>
d_{\mathrm{near}}
>0
\]

\[
\boxed{
d_{\mathrm{far}},\ d_{\mathrm{near}}
\text{ 为预设的距离参数}
}
\]

这里各位置的作用可以简单理解为：

- **FAR**：在离树体较远的位置完成主要作业准备；
- **NEAR**：进入切割前的近距离准备区域；
- **CUT_IN**：保持 CUT_ALIGN，沿 \(\mathbf u_{\mathrm{insert}}\) 低速向 CONTACT 推进。

因此，UAV 的接近方向和作业位置并不是预先固定，而是根据当前枝条、机械臂和链锯几何实时生成。

---

# 6. 问题五：搭建可分段执行、恢复、一键执行的作业架构

前面四个问题已经回答了：

```text
切哪个平面
  ↓
链锯怎么摆
  ↓
有效切割区域要落在哪里
  ↓
UAV 从哪里、怎么接近
```

最后还需要把这些几何结果组织成真正可运行的任务。

当前架构的核心是：

\[
\boxed{
\text{分段调试}
+
\text{状态恢复}
+
\text{一键执行}
}
\]

## 6.1 扫描、锁定与执行

切割前先通过 ARC_SCAN 改变观察视角，为阶段 C 的多视角融合提供更多数据：

\[
\psi_{\mathrm{scan}}
\in
[
\psi_0-\Delta\psi,\,
\psi_0+\Delta\psi
]
\]

当前 \(\Delta\psi\approx60^\circ\)。

扫描后执行 TARGET_LOCK，将实时模型冻结为：

\[
\boxed{
\mathcal B_{\mathrm{lock}}
=
\left\{
\mathbf p_0^*,
\mathbf d^*,
L^*,
r^*
\right\}
}
\]

这样可以避免执行过程中 BranchModel 的小幅波动不断改变切割几何。

## 6.2 机械臂姿态与安全撤退

机械臂保留三类姿态：

- **HOME**：飞行、返航、降落；
- **PREWORK**：固定过渡姿态；
- **CUT_ALIGN**：根据当前枝条几何动态计算的作业姿态。

进入作业区：

```text
HOME
→ PREWORK
→ CUT_ALIGN
```

切割完成后：

```text
CUT
→ RETREAT_NEAR
→ RETREAT_FAR
→ PREWORK
→ HOME
```

即：

\[
\boxed{
\text{先退出树体附近}
\rightarrow
\text{再收机械臂}
}
\]

避免在目标附近直接大幅折叠机械臂。

切割成功后同时执行 perception reset，清除已脱落枝条的历史融合模型，使感知状态与物理环境保持一致。

## 6.3 分段调试与一键执行

主要阶段均可单独调用：

```text
takeoff / observe / scan / lock / far
prework / cutalign / near / cut
retreat / armhome / land
```

直接使用：

```text
start
```

即可运行完整任务，同时保留 `pause / resume / retry / reset / abort` 等基本恢复功能。

---

# 7. 坐标系转换

感知和几何规划使用 world/ENU，而 PX4 使用 NED，因此目标位置不能直接发送给 PX4。

当前位置映射为：

\[
x_{\mathrm{PX4}}
=
y_{\mathrm{world}}
+\Delta x
\]

\[
y_{\mathrm{PX4}}
=
x_{\mathrm{world}}
+\Delta y
\]

\[
z_{\mathrm{PX4}}
=
-z_{\mathrm{world}}
+\Delta z
\]

其中 \([\Delta x,\Delta y,\Delta z]\) 由启动时 world odometry 与 PX4 odometry 在线计算。

yaw 同样进行对应转换：

\[
\psi_{\mathrm{PX4,target}}
=
\operatorname{wrap}
\left(
\psi_{\mathrm{bias}}
-
\psi_{\mathrm{world,target}}
\right)
\]

这样上层感知与几何规划可以统一在 world 坐标中完成，只在最终发送 PX4 指令时做转换。

---

---

# 8. 一次完整自动实验结果

一次代表性实验中，锁定得到：

\[
\mathbf p_0
=
[4.056,\;0.082,\;3.023]\;m
\]

\[
\mathbf d
=
[0.854,\;-0.192,\;-0.483]
\]

\[
L=0.291\;m,\qquad
r=0.035\;m
\]

程序进一步计算得到：

```text
normal alignment = 1.0000
insertion dir    = [0.457, 0.722, 0.520]
UAV yaw          = 110.5 deg
arm CUT joints   = [-102.5, 57.7, 42.9] deg
TCP ref error    = 0.009 m
```

其中：

\[
\boxed{
\eta_{\mathrm{align}}
=
1.0000
}
\]

说明该次实验中链锯导板法向与枝条主轴完成了预期对齐；TCP 参考误差约为：

\[
e_{\mathrm{TCP}}
\approx9\,mm
\]

最终完整任务成功运行：

```text
TAKEOFF
→ OBSERVE
→ SCAN
→ TARGET_LOCK
→ FAR
→ PREWORK
→ CUT_ALIGN
→ NEAR
→ CUT
→ RETREAT
→ HOME
→ RETURN_HOME
→ LANDING
→ DONE
```

说明当前已经能够完成：

\[
\boxed{
\text{BranchModel}
\rightarrow
\text{几何计算}
\rightarrow
\text{UAV--Arm 切割动作}
}
\]

的完整最小闭环。

---

# 9. Stage D 阶段总结

Stage D 表面上完成的是一个 Offboard 自动切割 Demo，但核心是把下面这条任务几何链第一次完整连了起来：

```text
BranchModel.direction
    ↓
切割平面
    ↓
UAV yaw + CUT_ALIGN
    ↓
链锯有效切割位置
    ↓
CONTACT base
    ↓
切入方向
    ↓
FAR / NEAR / CUT_IN
    ↓
自动切割与安全恢复
```

换句话说，本阶段依次解决了：

1. **枝条应该沿哪个平面切；**
2. **链锯导板应该怎样摆到这个平面；**
3. **链锯真正参与切割的区域应该落在哪里；**
4. **UAV 应从哪个方向接近，并怎样布置 FAR / NEAR / CUT_IN；**
5. **如何把这些几何结果组织成可调试、可恢复、可一键运行的任务。**

因此，本阶段形成了：

\[
\boxed{
\text{Sense}
\rightarrow
\text{Model}
\rightarrow
\text{Plan}
\rightarrow
\text{Act}
\rightarrow
\text{Recover}
}
\]

最小自主切割闭环。

---

# 附录 A：完整启动与测试指令

## A.1 编译

```bash
cd ~/UAV_lumberjack/ros2_ws

source /opt/ros/humble/setup.bash
source ~/ws_ros2/install/setup.bash

colcon build \
  --packages-select \
  uav_lumberjack_perception \
  uav_lumberjack_control \
  --symlink-install \
  --allow-overriding uav_lumberjack_control

source install/setup.bash
```

---

## A.2 终端 1：启动仿真

```bash
cd ~/UAV_lumberjack/ros2_ws

source /opt/ros/humble/setup.bash
source ~/ws_ros2/install/setup.bash
source install/setup.bash

ros2 launch uav_lumberjack_control uav_lumberjack_validation.launch.py
```

---

## A.3 终端 2：启动机械臂控制器

```bash
cd ~/UAV_lumberjack/ros2_ws

source /opt/ros/humble/setup.bash
source ~/ws_ros2/install/setup.bash
source install/setup.bash

ros2 run uav_lumberjack_control arm_controller
```

---

## A.4 终端 3：启动枝条感知

```bash
cd ~/UAV_lumberjack/ros2_ws

source /opt/ros/humble/setup.bash
source ~/ws_ros2/install/setup.bash
source install/setup.bash

ros2 launch uav_lumberjack_perception perception.launch.py
```

---

## A.5 终端 4：启动 RViz

```bash
cd ~/UAV_lumberjack/ros2_ws

source /opt/ros/humble/setup.bash
source ~/ws_ros2/install/setup.bash
source install/setup.bash

rviz2 -d ~/UAV_lumberjack/sim/default.rviz \
  --ros-args -p use_sim_time:=true
```

---

## A.6 终端 5：启动 Offboard Cutting Demo

```bash
cd ~/UAV_lumberjack/ros2_ws

source /opt/ros/humble/setup.bash
source ~/ws_ros2/install/setup.bash
source install/setup.bash

ros2 run uav_lumberjack_control offboard_cutting_demo \
  --ros-args \
  --params-file \
  "$(ros2 pkg prefix --share uav_lumberjack_control)/config/offboard_cutting_demo.yaml"
```

---

## A.7 完整自动测试

在 Offboard Cutting Demo 终端依次执行：

```text
status
sethome
start
```

---

## A.8 分段调试命令

需要单独测试某个阶段时，可以使用：

```text
takeoff
observe
scan
lock
far
prework
cutalign
near
cut
retreat
armhome
land
```

任务管理：

```text
status
pause
resume
retry
reset
reset_target
sethome
abort
params
help
```
