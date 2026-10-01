# UAV_lumberjack 项目交接说明
**日期：2026-09-30**  
**用途：用于切换到新的 ChatGPT 对话窗口后，快速恢复全部项目上下文。**

> **非常重要：**
> 当前对话最后用户上传了 `UAV_lumberjack.zip`，但用户明确要求“先别阅读这个文件”。  
> 因此，本交接文件 **没有读取、解压、检查或依据该压缩包内容生成任何结论**。  
> 新对话中也不要假设该压缩包已经被审查过；只有在用户明确要求后再检查。

---

# 0. 与用户协作方式

用户是控制工程方向研究生，目前在做 UAV + 机械臂 + 电锯 + RGB + MID360 的树枝切割仿真/研究项目。

用户非常偏好：

- **1 对 1 导师式推进**
- 每次只推进 **1～3 个小步骤**
- 要先说明原理，再给明确指令，再让用户验证
- **所有测试都必须从零给完整启动命令**
- 不要默认“某个程序已经开着”
- 大改 Python / C++ / SDF / launch 时，最好直接给完整可下载文件
- 小改一两行时可以手改
- 完成一个阶段后要“冻结”，不要反复回头调
- 结论不要重复说很多遍
- 技术解释要同时包含：
  - 专业表述
  - 用户能直接给导师解释的直观表述

用户喜欢称呼助手“GPT 小宝 / GPT 宝”，语气可以自然、亲切，但技术上要严谨。

---

# 1. 项目总体目标

项目：**UAV lumberjack（无人机树枝切割系统）**

最终目标大致是：

```text
无人机进入作业区
↓
搜索 / 观察树冠
↓
RGB + LiDAR 感知树枝
↓
得到树枝三维几何模型
↓
识别风险枝条 / 选择切割目标
↓
规划 UAV 工作位姿
↓
悬停
↓
机械臂运动到切割位姿
↓
启动电锯
↓
切断树枝
↓
验证切割结果
↓
更新场景
↓
继续下一目标
```

当前阶段刚完成感知 Stage C 的主体，准备做最终工程收尾。

---

# 2. 软件与环境

用户环境：

- Ubuntu 22.04.5
- ROS2 Humble
- PX4 1.16.2
- Gazebo Harmonic
- gz-sim 8.15.0
- ros_gz_bridge gzharmonic
- Micro-XRCE-DDS-Agent 3.0.1

主要路径：

```bash
~/UAV_lumberjack
~/PX4-Autopilot
~/UAV_lumberjack/ros2_ws
```

`.bashrc` 已经会自动 source：

```bash
/opt/ros/humble/setup.bash
~/UAV_lumberjack/ros2_ws/install/setup.bash
```

但是：

> **新建 package / executable 之后，当前已经打开的终端仍然要重新 source install/setup.bash。**

---

# 3. Python / OpenCV 环境——绝对不要动

用户当前环境：

```text
/usr/bin/python3
NumPy 2.2.6（user pip）
opencv-python 4.13.0.92（user pip）
```

ROS2 Humble 的 `cv_bridge` 与 NumPy 2 不兼容，曾出现：

```text
_ARRAY_API not found
```

因此 Step13 已经明确：

> **禁止改 NumPy / OpenCV 版本。**

项目中的图像读取采用：

```python
sensor_msgs/Image.msg.data
+
np.frombuffer(...)
```

**不使用 cv_bridge。**

这点后续绝对不要建议用户“降级 NumPy”之类。

---

# 4. 项目阶段划分

## Stage A
MVP，UAV + 机械臂 + 电锯基本功能。

已有总结：

```text
A_MVP_阶段性总结.md
```

---

## Stage B
V2 仿真平台。

**截至 Step12 已完成并冻结。**

---

## Stage C
感知。

当前核心是：

> **Step13：基于 RGB–LiDAR 融合的目标枝条三维感知与几何参数提取**

当前 Step13 主体已经跑通，正在做最终工程收尾。

---

## Stage D
后续准备做：

- UAV 工作位姿生成
- PX4 Offboard 自动接近
- 悬停
- 根据 `branch_model` 规划机械臂 / 切割

还没正式开始。

---

# 5. 机械臂与电锯系统

机械臂自由度：

- UAV yaw 代替水平 J1
- J2 shoulder pitch
- J3 elbow pitch
- J4 wrist roll

全系统物理自由度：

```text
[x, y, z, roll, pitch, yaw, J2, J3, J4]
```

局部任务规划：

```text
[psi_UAV, q2, q3, q4]
```

关节范围：

```text
J2 : -165° ~ +15°
J3 : -150° ~ +150°
J4 : -180° ~ +180°
```

HOME：

```text
J2 = -30°
J3 = -150°
J4 = -90°
```

PREWORK：

```text
J2 = -60°
J3 = +60°
J4 = 0°
```

已验证低位切割姿态：

```text
(-45, -45, 0)
(-30, -60, 0)
```

机械臂控制器启动：

```bash
cd ~/UAV_lumberjack/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run uav_lumberjack_control arm_controller
```

---

# 6. Step10：切割逻辑（冻结）

路径：

```text
~/UAV_lumberjack/sim/step10_v2_tree
```

目标枝条：

```text
红色
长度 GT = 0.30 m
半径 GT = 0.038 m
```

切断服务：

```text
/target_branch/detach
```

接触：

```text
/lumberjack_arm/target_contact
```

切割进度：

```text
/lumberjack_arm/cut_progress
```

切割规则：

- 电锯阈值：800 rpm
- 有效接触时间：>= 0.8 s
- gap grace：0.25 s
- 500 rpm 不切
- 1000 rpm 能切
- 已验证

---

# 7. Step11：RGB 相机（冻结）

相机：

```text
1280 × 720
30 Hz
HFOV ≈ 90°
VFOV ≈ 58.7°
```

topic：

```text
/camera/image_raw
/camera/camera_info
```

编码：

```text
rgb8
```

frame：

```text
x500_lumberjack/camera_link/imager
```

CameraInfo：

```text
fx ≈ 640
fy ≈ 640
cx = 640
cy = 360
```

---

# 8. Step12：MID360（冻结）

topic：

```text
/mid360/scan
/mid360/scan/points
/mid360/points
```

ROS PointCloud2：

```text
/mid360/points
```

frame：

```text
x500_lumberjack/mid360_link/mid360_gpu_lidar
```

参数：

```text
360° horizontal
vertical: -7° ~ +52°
360 × 60
10 Hz
near 0.1 m
far 40 m
```

理论点率：

```text
216k points/s
```

PointCloud2 字段：

```text
x         offset 0   float32
y         offset 4   float32
z         offset 8   float32
intensity offset 16  float32
ring      offset 24
```

---

# 9. Step13 总体目标

标题：

> **基于 RGB–LiDAR 融合的目标枝条三维感知与几何参数提取**

最终模型：

\[
\mathcal B = \{\mathbf p_0,\mathbf d,L,r\}
\]

其中：

- `p0`：枝条中心
- `d`：枝条主轴方向
- `L`：长度
- `r`：半径

最终给 Stage D 使用。

---

# 10. Step13 感知链路

整体逻辑：

```text
RGB 图像
↓
红色枝条 Mask
↓
LiDAR → Camera 投影
↓
M(u,v)>0
↓
保留对应三维点
↓
深度聚类
↓
去除背景穿透
↓
TARGET 状态机
↓
world 坐标转换
↓
多视角在线融合
↓
PCA + 圆柱拟合
↓
p0 / d / L / r
↓
/perception/branch_model
```

---

# 11. 用户已掌握的概念

这些解释风格在后续 Step13.md 中应保留。

## 语义
用户的直观理解：

> “判断这是什么 / 是不是我要的目标。”

当前红色分割相当于最简单的目标语义识别。

未来 YOLO 可以替代。

---

## Mask
用户理解：

> 图像过滤 + 二值化。

```text
目标像素 = 255
无关像素 = 0
```

专业表述：

> binary target segmentation

---

## RGB–LiDAR 对应关系

\[
(u_i,v_i)\leftrightarrow p_i^{3D}
\]

用户理解：

> 图像二维像素与 LiDAR 三维点之间的对应关系。

条件：

\[
M(u_i,v_i)>0
\]

含义：

> LiDAR 点投影到图像以后，看看是不是落在白色目标区域里。

如果是，就保留原来的三维点。

---

# 12. LiDAR → Camera

用户偏好简单形式：

\[
\boxed{\mathbf p_C=R_{CL}\mathbf p_L+t_{CL}}
\]

不要在报告里首先上齐次矩阵。

Gazebo 相机坐标：

```text
+X forward
+Y left
+Z up
```

投影：

\[
u=c_x-f_xY_C/X_C
\]

\[
v=c_y-f_yZ_C/X_C
\]

---

# 13. PCA

必须写全称：

> **PCA（Principal Component Analysis，主成分分析）**

专业表述：

> PCA 通过分析目标点云的协方差矩阵，提取最大特征值对应的特征向量，从而得到枝条点云第一主方向 \(\mathbf d\)。

用户直观理解：

> “算那个向量。”

再准确一点：

> “把一堆点变成一根有方向的轴。”

协方差：

\[
C=\frac1N\sum_i(p_i-\bar p)(p_i-\bar p)^T
\]

最大特征值对应特征向量：

\[
v_1 \rightarrow \mathbf d
\]

注意：

> PCA 主要直接给 `d`，不是自动给 `p0/L/r`。

`p0` 用点云中心/轴向几何得到。  
`L` 用点投影到 `d` 以后求轴向范围。  
`r` 用垂直主轴的横截面几何拟合。

---

# 14. Step13 软件包

感知包：

```text
~/UAV_lumberjack/ros2_ws/src/uav_lumberjack_perception
```

主要 executable：

```text
sensor_check
red_branch_detector
lidar_camera_projection
target_branch_cloud
branch_pca
odom_to_tf
target_cloud_world
multi_view_fusion
branch_pca_fused
```

独立感知 launch：

```text
step13_perception.launch.py
```

用户明确要求：

> **感知 launch 与仿真 launch 分开，不要合并。**

仿真 launch：

```text
uav_arm_step13.launch.py
```

---

# 15. Step13 已解决的重要问题

## 15.1 cv_bridge
NumPy2 ABI 不兼容。

解决：

> 完全绕开 cv_bridge。

---

## 15.2 投影性能
最早只有 ~5.5 FPS。

优化后：

```text
~9.3 FPS
~3 ms process
```

关键优化：

- 相机仅保存 latest msg
- 只在 LiDAR ~10Hz callback 做图像转换
- static TF cache
- NumPy structured PointCloud2
- display 0.75
- Best Effort

---

## 15.3 背景穿透

现象：

> Mask 只能判断二维像素，不知道同一视线方向哪个深度才是目标。

所以目标后面的树干/背景 LiDAR 点也会落入 Mask。

用户报告可用表述：

> Mask只判断二维像素是否属于目标区域，不能区分同一视线方向不同深度的三维点，因此目标后方的树干/背景点也可能投影到同一白色Mask区域，形成背景穿透。

解决：

> **1D 深度聚类**

不是硬编码“目标距离”。

---

# 16. TARGET 状态机

状态：

```text
LOST
↓
REACQUIRING
↓
TRACKING
```

最新版本：

```text
TRACKING -> LOST
连续 5 帧 missing
```

约：

```text
0.5 s
```

重新捕获：

```text
REACQUIRING
需要 3 帧确认
```

REACQUIRING 允许：

```text
1 帧短暂 missing
```

只有：

```text
连续 2 帧 missing
```

才回 LOST。

非常重要：

> grace 期间状态可能仍保持 TRACKING / REACQUIRING，但输出的是空 target cloud，不会继续输出旧几何，所以不会污染下游。

最近测试中存在反复：

```text
TRACKING -> LOST
LOST -> REACQUIRING
REACQUIRING -> TRACKING
```

用户判断：

> 很可能目标刚好处于 RGB–LiDAR 联合有效视野边缘，无人机位置轻微振荡导致目标在有效/无效区域之间切换。

当前决定：

> **暂不继续调 TARGET。**

这不再视为主要 bug。

---

# 17. 世界坐标 TF

Gazebo OdometryPublisher：

```text
/model/x500_lumberjack/base_link_odometry
```

world/base_link。

`odom_to_tf.py`：

```text
world -> base_link
```

并保留原始 odometry timestamp。

`target_cloud_world.py`：

```text
/perception/target_branch_cloud
↓
world exact-time transform
↓
/perception/target_branch_cloud_world
```

已解决 TF extrapolation：

- exact timestamp
- 如果对应 TF 还没到：
  - queue
  - 每 10 ms retry
  - 最多等 100 ms
  - max pending 3
- target invalid 时清 pending
- 不使用“latest TF”作弊
- 不篡改时间戳

此逻辑已经冻结。

---

# 18. 多视角融合

手动多视角融合已经完成。

历史实验：

## Group 1

```text
n = 105
L = 0.3088 m
length error = 2.94%

r = 0.0374 m
radius error = 1.70%

rms = 0.0043 m
```

## Group 2

```text
n = 114
L = 0.3018 m
length error = 0.60%

r ≈ 0.0352~0.0353 m
radius error ≈ 7.2%

rms = 0.0076 m
```

结论：

> 多视角最明显改善的是半径/横截面，不只是长度。

---

# 19. 在线融合失败与最终方案

最早尝试：

```text
固定 0.75 s
+
5 mm voxel
+
直接累积
```

失败：

```text
fused 点数不断增长
L 漂到 ~0.95 m
r 漂到 ~0.094 m
```

原因：

> 同一表面的小抖动 + TF/姿态误差会让点跨 voxel，因此“新 voxel”不等于“新表面”。

因此加入：

```text
ACCEPT
IGNORE
REJECT
```

---

# 20. 当前在线融合逻辑（冻结）

判断逻辑：

- PCA 主轴一致性
- 中心一致性
- radial corridor
- axial gate
- novelty distance
- novelty ratio
- voxel downsample

行为：

```text
ACCEPT
= 是目标 + 有新表面
```

```text
IGNORE
= 是目标 + 基本重复
```

```text
REJECT
= 几何不可信
```

用户已经理解这三个概念。

---

# 21. 自动 Bootstrap

之前问题：

> 第一帧可能质量差，却被直接当成 reference，导致之后正常点云一直 axis_mismatch，需要人工 reset。

已修复。

当前：

```text
bootstrap_confirm_frames = 3
bootstrap_min_points = 20
bootstrap_min_axis_ratio = 5
bootstrap_max_axis_angle_deg = 15°
bootstrap_max_center_distance = 0.12 m
```

逻辑：

```text
连续 3 帧稳定
↓
主轴一致
↓
点数足够
↓
PCA 质量足够
↓
BOOTSTRAP LOCKED
```

如果 bootstrap 后早期连续出现严重 mismatch：

```text
startup_rebootstrap_rejects = 5
```

则：

> 自动清掉早期错误 reference，重新 bootstrap。

所以现在：

> **不需要人工 `/perception/multiview/reset` 才能正常启动。**

这个问题已经解决。

---

# 22. 端点防漂移

最终在线融合采用：

> **stable reference axis + confirmed adaptive endpoints**

核心：

- reference axis 稳定
- 首帧提供初始 axial envelope
- 后续视角提出端点扩展
- 扩展必须多帧重复确认
- 当前确认帧数：3
- 每次扩展有限制
- 防止长度无限增长
- 又避免“第一帧太短导致永远锁死”

此逻辑已经冻结，不要再随便调。

---

# 23. 在线融合代表性结果

之前最好结果之一：

```text
n = 171
L = 0.3021 m
length error ≈ 0.7%

r = 0.0371 m
radius error ≈ 2.4%

rms = 0.0061
```

最近 BranchModel 一次正常输出：

```text
center:
  x: 4.058852195739746
  y: 0.08462956547737122
  z: 3.023390293121338

direction:
  x: 0.8850579857826233
  y: -0.13912220299243927
  z: -0.44420427083969116

length: 0.2931735888123512
radius: 0.03633233745220351
fit_rms: 0.006079646490699778
point_count: 170
```

对应 GT：

```text
L_GT = 0.300 m
r_GT = 0.038 m
```

误差约：

```text
length ≈ 2.3%
radius ≈ 4.4%
```

---

# 24. 当前终端日志设计

用户之前嫌终端非常乱，因此已经做了多轮简化。

现在正常重点看：

```text
[TARGET]
[TF]
[FUSION]
[MODEL]
```

launch executable 使用短名称：

```text
tfsrc
mask
target
worldtf
single
fusion
model
```

例如：

```text
[target-3]
[fusion-6]
[model-7]
```

其中 `-3/-6/-7` 是 ROS2 launch 自动进程序号，不是节点名本体。

当前日志风格：

```text
[FUSION]
  decision        : ACCEPT
  novel points    : 18 / 89
  novelty         : 20.2 %
  model           : 172 -> 190 pts
```

```text
[MODEL]
  points          : 190
  center p0       : [...]
  direction d     : [...]
  length L        : ...
  radius r        : ...
  fit RMS         : ...
```

TARGET 稳定 TRACKING 的周期日志已经降到 DEBUG。

正常终端不会再每两秒刷：

```text
points
depth
fps
```

---

# 25. BranchModel 最终 ROS2 输出接口

新建接口包：

```text
uav_lumberjack_interfaces
```

消息：

```text
msg/BranchModel.msg
```

定义：

```text
std_msgs/Header header

bool valid

geometry_msgs/Point center
geometry_msgs/Vector3 direction

float64 length
float64 radius
float64 fit_rms

uint32 point_count
```

已成功：

```bash
ros2 interface show \
  uav_lumberjack_interfaces/msg/BranchModel
```

---

# 26. perception 已接入 BranchModel

最终 topic：

```text
/perception/branch_model
```

成功测试：

```bash
ros2 topic echo /perception/branch_model --once
```

输出：

```text
header:
  stamp:
    sec: 131
    nanosec: 0
  frame_id: world

valid: true

center:
  x: 4.058852195739746
  y: 0.08462956547737122
  z: 3.023390293121338

direction:
  x: 0.8850579857826233
  y: -0.13912220299243927
  z: -0.44420427083969116

length: 0.2931735888123512
radius: 0.03633233745220351
fit_rms: 0.006079646490699778
point_count: 170
```

意义：

> `[MODEL]` 是给人看的。  
> `/perception/branch_model` 是给后续程序看的。

Stage D 后面可以直接订阅：

```text
msg.center
msg.direction
msg.length
msg.radius
```

---

# 27. BranchModel valid 语义

当前：

```text
valid: true
```

代表：

> 当前 fused model 可供下游使用。

如果：

- fused cloud 太少
- geometry 计算失败

则发布：

```text
valid: false
```

这样后续控制不会拿无效模型去规划。

---

# 28. build 过程中已解决的问题

接口包第一次 `--symlink-install` 时出现：

```text
failed to create symbolic link ...
existing path cannot be removed: Is a directory
```

解决方法：

```bash
cd ~/UAV_lumberjack/ros2_ws
source /opt/ros/humble/setup.bash

rm -rf build/uav_lumberjack_interfaces
rm -rf install/uav_lumberjack_interfaces
```

然后：

```bash
colcon build \
  --packages-select uav_lumberjack_interfaces \
  --symlink-install
```

成功。

然后 perception：

```bash
colcon build \
  --packages-select uav_lumberjack_perception \
  --symlink-install
```

也成功。

---

# 29. 当前常用完整启动命令

## 终端 1：仿真

```bash
cd ~/UAV_lumberjack/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch uav_lumberjack_control uav_arm_step13.launch.py
```

---

## 终端 2：感知

```bash
cd ~/UAV_lumberjack/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch uav_lumberjack_perception step13_perception.launch.py
```

---

## 终端 3：RViz

```bash
cd ~/UAV_lumberjack/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

rviz2 --ros-args -p use_sim_time:=true
```

Fixed Frame：

```text
world
```

---

## 查看最终模型接口

```bash
cd ~/UAV_lumberjack/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /perception/branch_model --once
```

---

# 30. RViz 已验证内容

用户已确认可以同时看到：

```text
/mid360/points
/perception/target_branch_cloud_world
/perception/target_branch_cloud_fused
```

以及：

```text
/perception/fused_branch_axis_marker
/perception/fused_branch_center_marker
/perception/fused_branch_length_marker
/perception/fused_branch_cylinder_marker
```

RViz 功能已经验证。

用户明确说：

> 不用继续美化 RViz，因为目前不是为了写论文。

所以不要再主动让用户调：

- 点大小
- 颜色
- 透明度
- 镜头

除非用户自己提出。

---

# 31. 当前已知小问题，但暂不影响主链

## 31.1 启动瞬间 TF not ready

曾出现一次：

```text
TF not ready:
"x500_lumberjack/camera_link/imager" ...
target_frame does not exist
```

属于：

> 节点启动时 TF 树还没完全起来的短暂时序问题。

后续正常运行，不阻断系统。

暂时不用专门修。

---

## 31.2 fused cloud QoS incompatible warning

曾出现：

```text
New subscription discovered ...
incompatible QoS
RELIABILITY
```

目前主链仍正常。

后续最终工程清理时检查。

---

# 32. 一个还没排查的重要飞控问题

用户明确要求后续提醒：

> **QGC 猛打虚拟摇杆 Yaw 时，无人机会突然下坠一下，然后再飞回原来的高度。**

可能相关：

- 快速 yaw 下姿态耦合
- 推力矢量变化
- 总升力下降
- PX4 高度控制补偿
- yaw rate / attitude controller
- 机体惯量 / motor model

但是：

> **目前还没正式排查。**

用户要求：

> Step13 收尾以后再检查。

新对话不要忘。

---

# 33. Step13 当前状态

已经完成：

```text
13.1 ✅
13.2 ✅
13.2.1 robust Mask ✅
13.3.1 LiDAR-camera projection ✅
13.3.2 Mask → target cloud ✅
13.3.3 depth clustering / background penetration ✅
13.3.4 target state machine ✅
13.4.1 PCA d ✅
13.4.2 p0 / L ✅
13.4.3 single-view r ✅
13.4.4 manual multi-view fusion ✅
13.4.5 online fusion ✅
TF exact-time retry queue ✅
automatic bootstrap ✅
terminal log cleanup ✅
13.5 RViz function verification ✅
13.6 BranchModel output interface ✅
```

---

# 34. 当前还剩的 Step13 收尾任务

**下一步就是最终工程体检。**

用户最后上传了：

```text
UAV_lumberjack.zip
```

但当前对话明确：

> **不要读取。**

因此新对话开始后，应先询问或等用户明确：

> “现在可以检查这个完整包了。”

再读取。

最终工程清理建议检查：

```text
uav_lumberjack_perception
uav_lumberjack_interfaces
```

重点：

- 目录结构
- setup.py
- package.xml
- CMakeLists.txt
- console_scripts
- launch
- 依赖
- 临时文件
- 重复代码
- __pycache__
- TODO
- license
- QoS
- 日志
- BranchModel
- TF
- TARGET state machine
- online fusion
- 不再使用的旧节点
- 是否有 debug / experiment 文件可删
```

原则：

> **不要再改已经验证稳定的算法，只做工程收尾。**

---

# 35. Step13 最终文档

已有较完整的 Markdown：

```text
branch_perception_updated.md
```

内容已经包含：

- RGB–LiDAR
- Mask
- projection
- background penetration
- depth clustering
- state machine
- PCA
- p0/L/r
- world TF
- manual fusion
- online fusion
- ACCEPT / IGNORE / REJECT
- anti-drift
- exact-time TF
- experiments
- paper方向

最后还需要再补：

- TARGET v5 hysteresis
- automatic bootstrap
- BranchModel topic
- interfaces package
- 最终运行方式
- Stage C 完成状态
- 当前已知 limitations
- Yaw 掉高问题放到后续，不属于 Step13

---

# 36. Stage C 冻结条件

等完整包最终检查结束后：

1. 修最后少量工程问题
2. 再测试一次：
   - sim
   - perception
   - branch_model
3. 更新最终 `Step13.md`
4. 宣布：
   > **Stage C 完成并冻结**

然后开始 Stage D。

---

# 37. Stage D 规划方向

Stage D 第一版建议不要一开始上复杂安全规划。

先做最简单闭环：

```text
branch_model
↓
选择目标
↓
生成 UAV 工作位姿
↓
PX4 Offboard 接近
↓
悬停
↓
机械臂到 pre-cut
↓
电锯启动
↓
接触
↓
切断
↓
机械臂回撤
```

后面再加入：

- safety distance
- line / tree / UAV collision
- reachability
- multiple branch selection
- obstacle clearing
- re-perception
- NBV

---

# 38. 后续研究/论文方向（仅概念）

可能研究标题方向：

> **面向 UAV 枝条切割任务的 RGB–LiDAR 在线鲁棒多视角三维建模方法**

核心链：

```text
深度聚类
↓
多视角融合
↓
增量建模
↓
信息有效性判断
↓
模型约束
↓
异常观测拒绝
↓
在线纠错
```

未来可能扩展：

```text
Theta = {p0, d, L, r, sigma, confidence}
```

以及：

```text
BOOTSTRAP
UPDATING
MATURE
REVALIDATING
```

还可以做：

- confidence
- uncertainty
- NBV
- real branch
- multi-segment branch
- cutting closed loop

但现在不要急着加。

---

# 39. PCL 立场

用户已经讨论过 PCL。

目前认知：

> PCL 是点云清洗/滤波/聚类/RANSAC 等工具库。

当前点云不复杂，NumPy + ROS2 已经够用。

因此：

> **目前不要为了“专业”硬上 PCL。**

以后真实点云更复杂再考虑。

---

# 40. 非常重要的冻结原则

以下内容已经花费大量时间验证：

- TF exact-time queue
- multi-view fusion
- ACCEPT / IGNORE / REJECT
- stable reference axis
- endpoint consensus
- automatic bootstrap
- target hysteresis
- RGB-LiDAR projection
- background penetration fix

后续除非出现明确、可复现 bug：

> **不要重新大改这些核心算法。**

优先继续推进系统闭环。

---

# 41. 新对话推荐第一句话

用户可以直接把这句话发给新 ChatGPT：

> 我们继续 UAV_lumberjack 项目。请先完整阅读我上传的交接说明，不要重新设计已经冻结的 Step13 算法。当前 Step13 的 RGB–LiDAR、多视角融合、TF、自动 Bootstrap、TARGET 状态机、BranchModel 接口都已经跑通。下一步先对我上传的完整工程包做最终工程体检和整理，然后更新最终 Step13.md、冻结 Stage C。之后还要排查 QGC 猛打 Yaw 时无人机短暂掉高的问题。请继续保持一步一步带我做，每次只推进 1～3 个小步骤，并给我完整启动命令。

---

# 42. 最终提醒给下一位助手

请不要：

- 降级 NumPy
- 引入 cv_bridge
- 合并 sim/perception launch
- 为了“美观”重新折腾 RViz
- 重新设计已冻结 fusion
- 把 TARGET 抖动简单当成重大 bug
- 忘记 BranchModel 已经成功发布
- 忘记 Yaw 猛转掉高问题
- 在用户没明确允许前读取本对话最后上传的 `UAV_lumberjack.zip`

优先：

1. 读交接说明
2. 等用户明确允许检查 zip
3. 做最终工程体检
4. 最终 Step13.md
5. Stage C freeze
6. 排查 Yaw 掉高
7. 再进入 Stage D

---

# 43. 当前最简状态总结

一句话：

> **Step13 的算法主体已经完成，最终 ROS2 `BranchModel` 接口也已经成功输出；现在只差完整工程包最终体检、文档更新和 Stage C 冻结。**

