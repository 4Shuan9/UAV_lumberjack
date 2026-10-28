# 阶段 D+：基于 Open3D 的感知链升级与环境理解

> **阶段 A：** 验证 UAV–Arm 基础飞行与机械臂动作可行性。  
> **阶段 B：** 建立 UAV–Arm、链锯、树枝与传感器仿真平台。  
> **阶段 C：** 基于 RGB–LiDAR 完成目标枝条三维感知与几何建模。  
> **阶段 D：** 将 `BranchModel` 接入 UAV–Arm Offboard 控制，跑通最小自动切割闭环。  
> **阶段 D+：** 在阶段 D 闭环的前提下，引入 Open3D，并补齐环境点云预处理与背景障碍树语义地图。

Stage D+ 阶段主要完成三件事：

1. **引入 Open3D 升级目标枝条多视角融合；**
2. **利用 Open3D 建立 MID360 自体过滤与起伏地面分割；**
3. **建立 RGB + LiDAR 背景障碍树语义地图，并完成 Offboard 回归验证。**

总体关系为：

```text
Stage D 已验证切割闭环
        ↓
Open3D 多视角融合
        ↓
环境 LiDAR 预处理
        ↓
背景障碍树语义地图
        ↓
原 BranchModel → Offboard Demo 继续正常运行
```

---

# 1. Stage D+ 的总体架构

阶段 D 的目标枝条链继续保留：

```text
RGB Mask
  ↓
目标枝条 3D 点
  ↓
world
  ↓
Open3D 多视角融合
  ↓
BranchModel
  ↓
Offboard Cutting Demo
```

D+ 新增环境感知链：

```text
MID360
  ↓
UAV / Arm 自体过滤
  ↓
world 点云
  ↓
地形分割
  ├─ ground
  └─ nonground
        ↓
    RGB + LiDAR
        ↓
   障碍树 Marker
```

两条链任务不同：

- **目标枝条**需要持续融合和较精确几何，服务切割；
- **背景障碍树**只需要稳定的位置和大致体量，服务环境理解。

---

# 2. 问题一：使用 Open3D 升级目标枝条融合

当前 `multi_view_fusion_node.py` 已使用 Open3D：

```python
import open3d as o3d
```

本阶段主要使用：

```text
voxel_down_sample()
compute_point_cloud_distance()
```

即体素化目标点云，并通过最近邻距离判断不同视角之间的空间支持关系。

本阶段没有使用 ICP、TSDF，也没有为了“框架升级”强行加入 PCL。

## 2.1 原 ROS 接口保持不变

输入：

```text
/perception/target_branch_cloud_world
```

输出：

```text
/perception/target_branch_cloud_fused
```

随后仍由：

```text
branch_pca_fused_node.py
```

生成：

```text
/perception/branch_model
```

因此阶段 D 的控制代码无需修改。

> **D+ 的核心原则是升级内部实现，同时保持已经验证的接口稳定。**

## 2.2 多帧稳定性

当前体素大小为：

```text
voxel_size = 0.005 m
```

即 5 mm。

融合不再把第一帧直接当作永久参考，而是先要求多帧几何一致，再建立参考枝条；枝条端点也需要重复观测支持后才逐步扩展。

```text
单帧目标点
    ↓
Open3D 体素化
    ↓
方向 / 中心 / 轴向一致性检查
    ↓
跨帧空间支持
    ↓
端点确认
    ↓
滚动融合点云
    ↓
PCA + 横截面几何
    ↓
BranchModel
```

这一步的意义主要是让目标点云融合更规范、可维护，并保持原切割链兼容。

需要注意：

> **仅仅“用了 Open3D”不能直接说明精度或速度一定明显提高，性能结论仍需要实验数据。**

---

# 3. 问题二：建立环境 LiDAR 预处理

<p align="center">
  <img src="/media/images/perception/cloud_point.jpg" width="85%">
</p>

原目标枝条感知只处理被视觉选中的局部区域，但环境理解需要区分：

```text
UAV / Arm 自体
地面
非地面环境
```

因此新增：

```text
environment_preprocessor
```

输入：

```text
/mid360/points
```

输出：

```text
/perception/lidar/points_self_filtered
/perception/lidar/points_ground
/perception/lidar/points_nonground
```

输出均转换到 `world`，原 `/mid360/points` 和目标枝条链保持不变。

## 3.1 UAV / Arm 自体过滤

MID360 会看到 UAV、机械臂和链锯自身。当前不是简单删除固定盒形区域，而是根据：

```text
SDF 几何
+
实时 J2 / J3 / J4
```

计算当前机械臂构型下的自体区域，再剔除相应回波。

过滤完成后利用 Gazebo odometry 把点云转换到 `world`，并保留原扫描时间戳。

## 3.2 起伏地面分割

Validation 地面近似平面，而 Agriculture 存在明显起伏，因此固定 `z` 阈值不再可靠。

当前 `terrain_ground_filter.py` 使用轻量 **PMF-inspired** 方法：

```text
低高程网格
   ↓
渐进形态学处理
   ↓
坡度连续性
   ↓
低地形锚点
   ↓
局部地面高度估计
   ↓
ground / nonground
```

这是针对当前工程实现的轻量算法，**不是 PCL 的编译版 PMF**。

地面候选范围由当前扫描的低高程分布自适应确定，因此同一套参数能够同时用于：

```text
validation_world
agriculture_world
```

Agriculture 起飞平台另外进行保护，不会被当地面删除。

这里采用一个安全原则：

> **当地形证据不足时，优先把未知点保留为障碍物，而不是错误删除。**

---

# 4. 问题三：背景障碍树语义地图

`points_nonground` 并不全部是树，其中还有岩石、建筑、电塔等结构。

因此新增：

```text
tree_semantic_mapper
```

当前封版为：

```text
Stage 3 V2.1
```

它没有使用 YOLO，而是组合：

```text
LiDAR 三维结构
+
RGB 颜色 / 纹理证据
+
多帧确认
+
树干位置修正
```

判断背景障碍树。

<p align="center">
  <img src="/media/images/perception/rviz_with_tree.png" width="90%">
</p>

## 4.1 RGB + LiDAR 分工

LiDAR 判断候选是否具有合理的低处支撑结构、树冠展开和三维尺度；RGB 提供植被颜色与纹理证据。

候选不会在单帧出现后立即注册，而是需要多个不同图像、持续一定时间且位置稳定后才确认。

## 4.2 一次确认、静态登记

背景障碍树不是当前切割对象，因此不需要持续做精细枝条拟合。

确认后使用：

```text
forest_tree_broadleaf_04
```

作为统一 RViz 树模型，并保持三轴等比例缩放。

输出：

```text
/perception/obstacle_trees/candidates
/perception/obstacle_trees/markers
```

已确认的障碍树在当前 ROS 进程中固定，不连续更新精细参数。

因此当前形成：

```text
目标枝条
→ 精细、多视角、持续融合

背景障碍树
→ 粗略、多帧确认、静态登记
```

两类任务不能混为一套模型。

---

# 5. 统一启动方式

当前 `perception.launch.py` 已将原目标枝条链和 D+ 环境感知整合到一个入口。

默认：

```text
environment     = true
tree_mapping    = false
self_filter_viz = true
```

直接运行：

```bash
ros2 launch uav_lumberjack_perception perception.launch.py
```

会启动：

```text
TF
RGB Mask
目标枝条点云
world 坐标转换
单帧 PCA
Open3D 多视角融合
BranchModel
环境点云预处理
自体过滤可视化
```

障碍树地图默认关闭，需要时启用：

```bash
ros2 launch uav_lumberjack_perception perception.launch.py tree_mapping:=true
```

这样既保持统一入口，也避免普通切割实验中增加不必要负载。

---

# 6. Stage D+ 验证结果

D+ 的验收重点是：

> **新增感知能力可以工作，同时不能破坏 Stage D 已验证的自动切割闭环。**

目前已经完成：

### Open3D 多视角融合

- Open3D 0.19.0 已接入；
- 5 mm voxel、多帧参考建立、端点确认和滚动融合已运行；
- `/perception/branch_model` 接口保持不变。

### 环境点云预处理

- Validation 平地已验证；
- Agriculture 起伏地形已验证；
- 自体、ground、nonground 可以分别显示；
- 少量零星伪点目前未证明会实质影响主任务。

### 背景障碍树地图

早期版本曾出现仓库 / 立柱误检和真实树漏检；V2 引入 RGB + LiDAR 联合判断，V2.1 再增加树干优先定位和时间 / 位置稳定确认后完成验收并封版。

最终再次运行 Offboard Demo，原自动切割闭环仍成功完成：

```text
[CUT-DEMO:DONE]
reason: landed + disarmed
[DONE] Offboard cutting demo complete.
Cut: progress=100% success=YES
```

因此当前已经验证：

```text
Open3D 融合
+
环境点云过滤
+
障碍树地图
+
Stage D Offboard 切割
```

能够在同一工程中共存。

---

# 7. 当前边界与下一阶段接口

D+ 仍然是**感知基础增强阶段**，并没有完成复杂枝条结构理解。

当前边界：

1. `BranchModel` 仍只描述单根目标枝条，不包含显式分叉拓扑；
2. Open3D 当前主要用于目标点云融合，而不是完整树骨架重建；
3. `points_nonground` 仍包含所有非地面物体，不能直接等同于枝条；
4. 障碍树地图只是粗略环境表示，不能代替目标枝条精细模型；
5. PCL 尚未成为当前运行依赖；
6. YOLO、MoveIt 2、TDBR、CLEAR / CUT 决策均不属于本阶段已完成内容。

但是 D+ 已为下一阶段准备了两个关键输入：

```text
/perception/target_branch_cloud_fused
→ 已知目标枝条的精细融合点云

/perception/lidar/points_nonground
→ 目标周围环境与潜在连接枝上下文
```

后续 TDBR / Stage E 可以直接在这两条数据基础上开发，而不需要破坏 Stage D。

---

# 8. Stage D+ 阶段总结

Stage D 完成：

```text
BranchModel
    ↓
切割几何
    ↓
UAV–Arm 自动切割
```

Stage D+ 补充：

```text
Open3D 目标点云融合
        +
MID360 环境点云预处理
        +
RGB + LiDAR 背景障碍树地图
```

最终形成：

```text
Target Sense
      +
Environment Sense
      ↓
更完整的三维感知基础
      ↓
继续服务已验证的 Offboard 切割闭环
```

本阶段没有强行加入更多复杂算法，而是完成从“只理解目标枝条”向“同时具备基础环境理解”的过渡，为后续 TDBR / Stage E 复杂枝条局部重建提供稳定输入。

---

# 附录 A：启动与测试指令

## A.1 编译

```bash
cd ~/UAV_lumberjack/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ws_ros2/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## A.2 启动仿真

Validation：
```bash
ros2 launch uav_lumberjack_control uav_lumberjack_validation.launch.py
```

Agriculture：
```bash
ros2 launch uav_lumberjack_control uav_lumberjack_agriculture.launch.py
```

## A.3 启动感知

启用背景障碍树地图：
```bash
ros2 launch uav_lumberjack_perception perception.launch.py tree_mapping:=true
```

## A.4 RViz2

```bash
rviz2 -d ~/UAV_lumberjack/sim/default.rviz \
  --ros-args -p use_sim_time:=true
```

`Fixed Frame = world`。

## A.5 机械臂控制器

```bash
ros2 run uav_lumberjack_control arm_controller
```

## A.6 Offboard

```bash
ros2 run uav_lumberjack_control offboard_cutting_demo \
  --ros-args \
  --params-file \
  "$(ros2 pkg prefix --share uav_lumberjack_control)/config/offboard_cutting_demo.yaml"
```

任务启动：

```text
status
sethome
start
```
