阶段 C ：基于 RGB–LiDAR 融合的目标枝条三维感知与几何参数提取小结

## 1. 小结

**本阶段目标：**
> **把“相机里看到的一根目标枝条”，转换成 UAV 后续定位、接近和切割能够直接使用的三维几何模型。**

当前采用 RGB-LiDAR 融合：

- RGB：负责判断“哪里是目标枝条”；
- LiDAR：负责提供对应位置的三维点；
- 点云处理：用于把 UAV 不同位置看到的同一根枝条叠加起来；
- 几何建模：进一步估计枝条的中心、方向、长度和半径。

整体流程为：

```text
单目相机采集
  ↓
RGB 图像处理
  ↓
目标 Mask
  ↓
LiDAR 点变换到 Camera 坐标系并投影到图像
  ↓
利用 Mask 筛选目标 LiDAR 点
  ↓
背景穿透 → 深度聚类
  ↓
目标检测状态机
  ↓
转换到 world 坐标系
  │
  ├─> 单帧 PCA / 圆柱几何拟合
  │
  └─> 多视角关键帧融合
          ↓
      多视角 PCA / 圆柱几何拟合
```

### 当前实验结果

- **单视角已经能够较稳定地估计枝条主方向和长度，但半径容易因为只看到局部圆柱表面而明显低估。**
- **手动多视角 world 点云融合已经证明，多视角能够明显补充枝条横截面的观测信息，两组三视角实验中半径误差分别降至 1.70% 和约 7.2%。**
- **在此基础上进一步完成轻量级在线增量融合：系统能够自动判断新观测属于 ACCEPT、IGNORE 还是 REJECT，不再依赖人为固定“必须采 3 个视角”。**
- **最新在线多视角实验最终得到 \(L=0.3021\,\mathrm{m}\)、\(r=0.0371\,\mathrm{m}\)，对应长度误差 0.7%、半径误差 2.4%。**
- **world 坐标转换中的 TF future extrapolation 问题已经通过 exact-time 短时重试队列解决，测试日志中不再出现 TF lookup failed / TF DROP。**

---

## 2. 最终目标：枝条几何模型

**先明确输出目标：最后不是只想得到一堆点，而是要把这些点变成后续规划真正能用的几何参数。**

将目标枝条定义为：

\[
\boxed{
\mathcal B=
\left\{
\mathbf p_0,\mathbf d,L,r
\right\}
}
\]

其中：

- \(\mathbf p_0\)：枝条几何中心；
- \(\mathbf d\)：枝条主轴方向单位向量；
- \(L\)：枝条有效长度；
- \(r\)：枝条半径。

Step13 最终希望完成的是：

```text
目标枝条点云
      ↓
{ p0, d, L, r }
      ↓
为后续切割规划奠定基础
```

### 2.1 操纵指令

终端 1：启动 Step13 仿真

```bash
cd ~/UAV_lumberjack/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_lumberjack_control uav_lumberjack_sim.launch.py
```

终端 2：启动 Step13 感知

```bash
cd ~/UAV_lumberjack/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_lumberjack_perception perception.launch.py
```

终端 3：启动 RViz

```bash
cd ~/UAV_lumberjack/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
rviz2 --ros-args -p use_sim_time:=true
```

开始一轮新的多视角实验前，清空旧融合数据：

```bash
ros2 service call /perception/multiview/reset std_srvs/srv/Trigger "{}"
```

当前在线融合已经自动运行，因此正常实验时**不再需要每到一个视角都手动执行 capture**。

典型实验方式：

```text
Reset
  ↓
View A 停留约 5 s
  ↓
移动到 View B
  ↓
移动到 View C / D / E
  ↓
系统自动判断 ACCEPT / IGNORE / REJECT
```

视角数量不固定为 3 个，实际可以根据新信息量使用约 1～5 个典型有效视角。

手动接口 `/perception/multiview/capture` 仍保留用于调试和对照实验，但已经不是正常在线融合的主要使用方式。

---

## 3. 传感器与 TF 基础验证

**确认相机、LiDAR 以及它们之间的位置关系**

### 3.1 单目相机

当前相机主要参数（基于真实设备设置）：

- 分辨率：1280 × 720；
- 帧率：约 30 Hz；
- 水平视场角：约 90°；
- 图像话题：`/camera/image_raw`；
- CameraInfo：`/camera/camera_info`。

TF 链：

```text
base_link
  ↓
camera_link
  ↓
x500_lumberjack/camera_link/imager
```

### 3.2 激光雷达

主要参数（根据 MID-360 配置）：

- 水平视场：360°；
- 垂直视场：约 \(-7^\circ\sim+52^\circ\)；
- 分辨率：360 × 60；
- 更新频率：约 10 Hz；
- 点云话题：`/mid360/points`。

TF 链：

```text
base_link
  ↓
mid360_mount_link
  ↓
mid360_link
  ↓
x500_lumberjack/mid360_link/mid360_gpu_lidar
```

---

## 4. 在图像中定位目标（Mask）

**在二维图像里告诉系统“哪些像素属于我要找的枝条”。**

当前仿真阶段暂不直接使用 YOLO，而是利用目标枝条的红色外观做简化语义识别。
> （“语义”可以直接理解为：判断“这是不是我要找的目标”。）

Mask 是一个二值图：
- 目标像素：255；
- 非目标像素：0。

因此：

\[
M(u_i,v_i)>0
\]

就表示像素 \((u_i,v_i)\) 落在目标区域。

<p align="center">
  <img src="target_branch_mask.png" width="85%">
</p>

### 4.1 Mask 鲁棒性改进

初版颜色条件较宽，在部分角度和光照下会把棕色树体误识别成目标。

最终采用：

\[
H\in[0,8]\cup[172,179]
\]

\[
S\ge110,
\qquad
V\ge30
\]

同时：

- `min_area = 80 px`；
- 只保留最大的有效连通区域。

主要输出：

```text
/perception/red_mask_raw    # 原始候选
/perception/red_mask        # 最终目标 Mask
/perception/red_debug       # 调试图
```

改进后在多个角度和不同光照下，目标识别明显更加稳定。

<p align="center">
  <img src="target_branch.png" width="85%">
</p>

---

## 5. 二维目标图像转换三维目标点云

**相机已经告诉“哪一块是目标”，现在要知道这块目标在三维空间中到底在哪里。**

### 5.1 LiDAR → Camera 坐标转换

首先把 LiDAR 点从 LiDAR 坐标系转换到 Camera 坐标系。

设：

- \(\mathbf p_L\)：LiDAR 坐标系中的三维点；
- \(\mathbf p_C\)：同一点在 Camera 坐标系中的坐标；
- \(R_{CL}\)：LiDAR 坐标系到 Camera 坐标系的旋转矩阵；
- \(t_{CL}\)：LiDAR 坐标系到 Camera 坐标系的平移向量。

则最核心的外参变换就是：

\[
\boxed{
\mathbf p_C
=
R_{CL}\mathbf p_L+t_{CL}
}
\]

这一步可以直观理解成：

> **先把 LiDAR 测到的点“搬到相机坐标系里”。**

### 5.2 Camera 三维点 → 图像像素

当前 Gazebo 相机坐标约定为：
- \(+X\)：前方；
- \(+Y\)：左；
- \(+Z\)：上。

若：

\[
\mathbf p_C
=
[X_C,Y_C,Z_C]^T
\]

则投影到图像：

\[
\boxed{
u
=
c_x-f_x\frac{Y_C}{X_C}
}
\]

\[
\boxed{
v
=
c_y-f_y\frac{Z_C}{X_C}
}
\]

于是建立：

\[
\boxed{
(u_i,v_i)
\leftrightarrow
\mathbf p_i^{3D}
}
\]

也就是说：

> 一个二维像素，现在可以找到它对应的 LiDAR 三维点；
> 反过来，一个 LiDAR 点也知道自己应该落在图像哪里。

<p align="center">
  <img src="Lidar2Camera.png" width="85%">
</p>

### 5.3 利用 Mask 筛选目标三维点

对每个 LiDAR 点完成投影后，检查：

\[
M(u_i,v_i)>0
\]

如果成立：

\[
\mathbf p_i^{3D}
\]

就被保留。

整个逻辑可以很直观地理解为：

```text
LiDAR 3D 点
   ↓
转换到 Camera 坐标系
   ↓
投影到图像像素 (u,v)
   ↓
这个像素在目标 Mask 里面吗？
   ├─ 是 → 保留对应 3D 点
   └─ 否 → 删除
```

这样就完成了从：

```text
二维目标区域
```

到：

```text
三维目标候选点云
```

<p align="center">
  <img src="target_cloud_outliers.png" width="85%">
</p>

### 5.4 深度聚类：去掉远处背景点

只靠 Mask 还不够。**Mask 只判断二维像素是否属于目标区域，并不能区分同一视线方向上不同深度的三维点**；
因此目标枝条后方的树干或其他背景点，也可能投影到同一白色 Mask 区域，形成“背景穿透”。

> 对候选点的深度做一维聚类。

当前主要约束：

- 相邻深度聚类间隔：0.20 m；
- 最小聚类点数：5；
- 有效目标深度范围：0.20～5.0 m；
- 单聚类最大深度跨度：0.60 m；
- 最大时序深度跳变：0.60 m；
- 最大质心跳变：0.75 m。

> 无需要写死枝条距离，亦能排除大部分远背景点。

### 5.5 目标有效性状态机

UAV 移动时由于俯仰角或横滚角，目标可能短暂离开视野或出现漏检。

为了避免一帧异常就把背景误认为新目标，加入：

```text
TRACKING
LOST
REACQUIRING
```

逻辑为：

```text
TRACKING
  ↓ 连续 3 帧缺失
LOST
  ↓ 再次出现候选
REACQUIRING
  ↓ 连续 3 帧确认一致
TRACKING
```

LiDAR 约 10 Hz，因此 3 帧约对应 0.3 s。

在 1～2 帧短暂缺失期间，不继续输出旧目标，而是输出空点云，避免下游继续使用已经过期的数据。

最终得到：

```text
/perception/target_branch_cloud
frame_id = base_link
```

---

## 6. 从点云提取枝条几何参数

**不再只看“点”，而是从这些点里面算出枝条的方向、中心、长度和粗细。**

<p align="center">
  <img src="target_branch_marker.png" width="85%">
</p>

### 6.1 PCA（Principal Component Analysis，主成分分析）提取主方向

对目标点：

\[
\mathbf p_i=[x_i,y_i,z_i]^T
\]

\[
i=1,\ldots,N
\]

先计算点云均值：

\[
\bar{\mathbf p}
=
\frac{1}{N}
\sum_{i=1}^{N}
\mathbf p_i
\]

构造协方差矩阵：

\[
\mathbf C
=
\frac{1}{N}
\sum_{i=1}^{N}
(\mathbf p_i-\bar{\mathbf p})
(\mathbf p_i-\bar{\mathbf p})^T
\]

其中：

\[
\mathbf C=
\begin{bmatrix}
C_{xx} & C_{xy} & C_{xz}\\
C_{yx} & C_{yy} & C_{yz}\\
C_{zx} & C_{zy} & C_{zz}
\end{bmatrix}
\]

特征分解：

\[
\lambda_1\ge\lambda_2\ge\lambda_3
\]

取最大特征值对应的特征向量：

\[
\boxed{
\mathbf d=\mathbf v_1
}
\]

作为枝条第一主方向。

直观理解就是：

> **PCA 把“一堆三维点”变成“一根最能代表这堆点延伸方向的轴”。**

### 6.2 计算中心 \(\mathbf p_0\) 和长度 \(L\)

把所有点沿主方向投影：

\[
s_i
=
(\mathbf p_i-\bar{\mathbf p})^T
\mathbf d
\]

取：

\[
s_{\min}=\min(s_i),
\qquad
s_{\max}=\max(s_i)
\]

则：

\[
\boxed{
L=s_{\max}-s_{\min}
}
\]

枝条几何中心：

\[
\boxed{
\mathbf p_0
=
\bar{\mathbf p}
+
\frac{s_{\min}+s_{\max}}{2}
\mathbf d
}
\]

因此这里的逻辑是：

```text
PCA 得到方向 d
      ↓
所有点沿 d 投影
      ↓
最小投影值 + 最大投影值
      ↓
得到长度 L 和中心 p0
```

仿真长度真值：

\[
L_{GT}=0.300\text{ m}
\]

单视角长度通常约为 0.28～0.31 m，多数情况下误差约 3%～6%。

### 6.3 横截面圆拟合得到半径 \(r\)

长度沿着主轴算，半径则要在主轴的垂直平面上计算。

在垂直于 \(\mathbf d\) 的平面中构造：

\[
\mathbf e_1,\mathbf e_2
\perp
\mathbf d
\]

将三维点投影到横截面：

\[
x_i
=
(\mathbf p_i-\mathbf p_0)^T
\mathbf e_1
\]

\[
y_i
=
(\mathbf p_i-\mathbf p_0)^T
\mathbf e_2
\]

拟合圆：

\[
x^2+y^2+Ax+By+C=0
\]

得到：

\[
c_x=-\frac{A}{2},
\qquad
c_y=-\frac{B}{2}
\]

\[
\boxed{
r=
\sqrt{
c_x^2+c_y^2-C
}
}
\]

仿真半径真值：

\[
r_{GT}=0.038\text{ m}
\]

单视角代表性结果：

| 指标 | 典型结果 | 误差 |
|:---:|:---:|:---:|
| \(L\) | 0.289～0.316 m | 2.6%～5.2% |
| \(r\) | 0.019～0.026 m | 31%～50% |

这里暴露出了当前单视角方法最明显的问题：

> **长度较为准确，但半径误差大。**

**原因分析：**
- 单个 LiDAR 视角只能看到圆柱朝向传感器的局部表面，横截面实际上只是部分圆弧，所以圆拟合容易低估真实半径。

**下一步：**
- 引入多视角融合的直接原因。

<!-- 图片建议：这里插入单帧 PCA 主轴 / 中心 / 长度 / 圆柱拟合 RViz 图，例如 images/single_view_geometry.png -->

---

## 7. world 坐标统一

**UAV 会移动，但树枝不动。故不同位置看到的点必须先转换到同一个固定坐标系，才能正确融合。**

建立：

```text
world
  ↓
base_link
  ├─> camera
  └─> mid360
```

动态 TF 后，将目标点从 `base_link` 转到 `world`：

\[
\boxed{
\mathbf p_W
=
R_{WB}\mathbf p_B+t_{WB}
}
\]

得到：

```text
/perception/target_branch_cloud_world
frame_id = world
```

测试中 UAV 改变位置和姿态后，静止枝条仍基本固定在 world 中，因此不同视角的点云具备叠加条件。

### 7.1 TF 时间同步问题与修复

在线融合过程中曾经频繁出现：

```text
TF lookup failed world <- base_link
Lookup would require extrapolation into the future
```

典型情况是：

```text
目标点云时间戳：t
最新 world -> base_link TF：t - 5~25 ms
```

原实现要求点云时间戳和 TF 精确对应，因此当 TF 比目标点云晚到几毫秒时，会直接丢弃该帧 world 点云。

最终没有采用“强行使用 latest TF”或人为修改 TF 时间戳的方法，而是在 `target_cloud_world` 中加入短时 exact-time 重试队列：

```text
目标点云 @ t
  ↓
查询 world <- base_link @ t
  ↓
有 TF？
  ├─ 是 → 立即转换
  └─ 否 → 暂时进入等待队列
              ↓
           每 10 ms 重试
              ↓
           最多等待 100 ms
              ↓
           exact-time TF 到达后再转换
```

同时，当目标进入 `LOST / REACQUIRING` 并输出空点云时，会立即清空等待队列，避免旧的有效目标在稍后重新发布。

修复后再次检查：

```bash
grep -E "\[TF DROP\]|TF lookup failed" /tmp/step13_perception.log
```

无输出，说明测试中不再出现之前的 future extrapolation 丢帧问题。

### 7.2 仿真时间与 RViz

Gazebo、ROS2 和 RViz 最终统一使用 `/clock` 与 `use_sim_time=true`。

RViz 曾出现长时间运行后卡死的问题。最终将目标点云、PCA 和几何 Marker 直接在 `world` 中发布，减少高频动态 TF 转换。修改后连续运行约 70 min，RViz 保持正常。

---

## 8. 多视角融合：从手动关键帧到在线增量建模

**既然一个角度只能看到枝条的一部分，就从不同位置观察同一根静止枝条，把不同方向看到的表面补到一起。**

当前多视角部分已经经历：

```text
手动关键帧融合
      ↓
固定时间自动累加
      ↓
发现重复点和漂移问题
      ↓
模型约束 + 新信息判断
      ↓
在线鲁棒增量融合
```

### 8.1 手动关键帧融合：先验证多视角是否真的有用

最初保留：

```text
/perception/multiview/capture
/perception/multiview/reset
```

每个视角转换到 `world` 后直接叠加，并做：

\[
\boxed{
P_{\mathrm{fused}}
=
\operatorname{Voxel}
\left(
P_1^W
\cup
P_2^W
\cup
\cdots
\cup
P_n^W
\right)
}
\]

voxel：

\[
5\text{ mm}
\]

这一步的目的不是最终在线运行，而是先验证：

> **多视角能不能补齐单视角看不到的圆柱表面。**

两组三视角实验已经证明：多视角对半径估计改善明显，因此继续向自动在线融合发展。

### 8.2 第一版自动融合失败：固定时间 + voxel 不够

最直接的自动化思路曾经是：

```text
每隔约 0.75 s
  ↓
把当前 world 目标点云加入 fused cloud
  ↓
做 5 mm voxel
```

这种方法实现简单，但实验很快暴露问题。

即使 UAV 基本停在同一个视角，传感器噪声、TF 微小误差和点云抖动也会让同一块表面落入新的 voxel：

```text
“进入了新 voxel”
≠
“真的看到了新的枝条表面”
```

结果是 fused cloud 会持续变厚、变长。

失败实验中曾出现：

```text
融合点数持续增长到 5000+
L ≈ 0.9553 m
r ≈ 0.0937 m
```

对于真值：

\[
L_{GT}=0.300\text{ m},
\qquad
r_{GT}=0.038\text{ m}
\]

显然已经发生严重模型漂移。

因此得到一个很重要的结论：

> **固定时间累加 + voxel 只能去重，不能判断一个新观测到底是不是新的有效表面信息。**

### 8.3 在线鲁棒融合：ACCEPT / IGNORE / REJECT

后续将融合从“定时累加”改成“先判断再决定”。

当前每一个候选观测只有三种结果：

```text
ACCEPT
  新视角提供了有效的新表面
  → 加入融合模型

IGNORE
  与当前模型基本重复
  → 不加入

REJECT
  与当前枝条几何明显不一致
  → 直接拒绝
```

主要判断包括：

- PCA 主轴方向是否与参考枝条一致；
- 当前点云中心是否偏离参考主轴过远；
- 点是否仍位于枝条允许的圆柱走廊中；
- 当前帧中有多少点距离 fused surface 足够远，能够认为是“新表面”。

定义新信息比例：

\[
\eta_{\mathrm{new}}
=
\frac{N_{\mathrm{novel}}}
{N_{\mathrm{valid}}}
\]

当前只有当新表面点数和比例同时达到阈值时才执行 ACCEPT。

因此正常状态会表现为：

```text
移动到新的有效视角
  ↓
ACCEPT
  ↓
模型更新

停在同一视角
  ↓
IGNORE
IGNORE
IGNORE
  ↓
模型不再增长
```

而当视角中的点云方向、位置明显不合理时：

```text
axis mismatch
center off axis
radial gate failed
  ↓
REJECT
```

### 8.4 轴向漂移问题：不能让 fused model 自己无限扩张

在线融合第一次加入模型约束后，曾经出现另一类问题：

> **如果每一轮都直接用新的 fused model 作为下一轮轴向边界，少量端点抖动会一轮一轮把长度向外推。**

表现为：

```text
0.30 m
  ↓
0.33 m
  ↓
继续增加
```

因此后续把“参考模型”和“融合点云”分开：

```text
Reference Model
  ↓
负责判断这是不是同一根枝条、允许的主轴方向和轴向范围

Fused Cloud
  ↓
负责保存真正吸收的新表面
```

### 8.5 第一帧范围固定过死的问题

完全固定第一帧范围也不合适。

如果第一帧本身只看到枝条的一部分：

```text
真实长度约 0.30 m
第一帧只看到约 0.21 m
```

后续即使看到了真实端点，也会因为固定轴向门控被挡掉。

因此最终采用：

> **参考主轴保持稳定，但枝条端点允许在多帧重复证据支持下逐步扩展。**

逻辑为：

```text
后续视角看到更远的轴向点
  ↓
不立即相信
  ↓
连续多帧得到相近端点证据
  ↓
确认该区域确实属于同一根枝条
  ↓
更新 confirmed axial range
```

当前使用 3 帧一致性确认端点扩展。

这样同时避免：

```text
边界完全跟着 fused cloud 跑
→ 越积越长

边界永久锁在第一帧
→ 永远偏短
```

最终变成：

\[
\boxed{
\text{稳定参考主轴}
+
\text{多帧确认端点}
+
\text{新表面判断}
+
\text{voxel 融合}
}
\]

整个过程仍然只使用 NumPy + ROS2 中已有的轻量计算，没有引入 ICP、TSDF 或复杂重型点云框架，便于后续向 RK3588 迁移。

### 8.6 单帧和多视角模型同时保留

当前仍保留两套独立估计：

```text
单帧：
target_branch_cloud_world
        ↓
branch_pca

在线多视角：
target_branch_cloud_fused
        ↓
branch_pca_fused
```

融合输出：

```text
/perception/target_branch_cloud_fused
frame_id = world
```

这样既可以观察当前单帧质量，也能实时比较多视角累计后的最终模型。

---

## 9. 多视角实验结果

**新增视角是否提高了几何参数估计精度？在线融合又能不能在长期运行时保持稳定？**

### 9.1 第一组三视角手动融合实验

| 视角 | 输入点数 |
|:---:|:---:|
| A | 50 |
| B | 36 |
| C | 21 |

融合：

```text
raw_total   = 107
voxel_total = 105
```

几何结果：

| 指标 | 估计结果 | 仿真真值 | 误差 |
|:---:|:---:|:---:|:---:|
| 融合点数 $n$ | 105 | — | — |
| 长度 $L$ | 0.3088 m | 0.300 m | 2.94% |
| 半径 $r$ | 0.0374 m | 0.038 m | **1.70%** |
| 拟合残差 | 0.0043 m | — | — |

### 9.2 第二组三视角手动重复实验

| 视角 | 输入点数 |
|:---:|:---:|
| A | 44 |
| B | 28 |
| C | 43 |

融合：

```text
raw_total   = 115
voxel_total = 114
```

几何结果：

| 指标 | 估计结果 | 仿真真值 | 误差 |
|:---:|:---:|:---:|:---:|
| 融合点数 $n$ | 114 | — | — |
| 长度 $L$ | 0.3018 m | 0.300 m | **0.60%** |
| 半径 $r$ | 0.0352～0.0353 m | 0.038 m | 约 7.2% |
| 拟合残差 | 0.0076 m | — | — |

这两组实验说明：

> **只要多个视角能够看到不同的圆柱表面，多视角融合就能明显改善单视角最薄弱的横截面/半径估计。**

### 9.3 在线增量融合稳定性实验

在线版本不再规定必须采集 3 个视角，而是让系统自己判断一个新观测是否值得融合。

一次完整实验中，模型经过不同有效视角后得到：

| 融合点数 \(n\) | 长度 \(L\) | 长度误差 | 半径 \(r\) | 半径误差 |
|:---:|:---:|:---:|:---:|:---:|
| 144 | 0.2838 m | 5.4% | 0.0375 m | **1.4%** |
| 162 | 0.2847 m | 5.1% | 0.0374 m | **1.7%** |
| 171 | **0.3021 m** | **0.7%** | 0.0371 m | **2.4%** |

最终：

\[
\boxed{
L=0.3021\text{ m}
}
\]

\[
\boxed{
r=0.0371\text{ m}
}
\]

相比仿真真值：

\[
L_{GT}=0.300\text{ m},
\qquad
r_{GT}=0.038\text{ m}
\]

最终误差分别约为：

\[
e_L=0.7\%
\]

\[
e_r=2.4\%
\]

### 9.4 重复视角抑制

模型达到：

```text
model = 162
```

后，在 UAV 保持当前视角观察较长时间时，新信息比例长期只有约：

```text
2.7% ~ 12.9%
```

融合节点连续输出：

```text
[FUSION IGNORE]
```

而模型点数保持不变。

这证明：

> **当前系统不会因为同一视角被连续重复观测，就一直向 fused cloud 中堆点。**

### 9.5 异常视角拒绝

实验中也实际出现：

```text
[FUSION REJECT] radial_gate ...
[FUSION REJECT] axis_mismatch ...
[FUSION REJECT] center_off_axis ...
```

说明当前在线融合不是简单依靠时间间隔自动累加，而是会拒绝与当前枝条几何明显不一致的观测。

### 9.6 单视角、手动多视角与在线多视角对比

| 方法 | 长度误差 | 半径误差 |
|:---|:---:|:---:|
| 单视角典型结果 | 2.6%～5.2% | 31%～50% |
| 手动三视角融合① | 2.94% | **1.70%** |
| 手动三视角融合② | **0.60%** | **约 7.2%** |
| 在线增量融合最终结果 | **0.7%** | **2.4%** |

**结果：**

> **多视角的主要价值仍然是补充横截面信息；在线版本在保留这种优势的同时，又加入了重复视角抑制、异常观测拒绝以及端点纠错，使模型可以随着有效新视角逐步更新，而不是盲目堆点。**

---

## 10. 遇到过的关键问题

**这一部分汇报时可以快速讲，主要说明当前系统不是只“跑通”，而是已经针对实际问题做过多轮修正。**

| 问题 | 解决办法 |
|---|:---:|
| 1. `cv_bridge` 与 NumPy 2 版本不兼容 | 不使用 `cv_bridge`，直接读取 `Image.data` 并通过 NumPy 转换 |
| 2. 红色 Mask 会把棕色树体识别为目标 | 收紧 HSV 范围并只保留主要连通区域 |
| 3. 远背景点穿透目标 Mask | 加入深度聚类与时序连续性约束 |
| 4. 单帧漏检导致目标状态抖动 | 3 帧丢失滞回 + 3 帧重捕获确认 |
| 5. PCA 主方向正负随机翻转 | 方向符号一致性 + 轻微平滑 |
| 6. UAV 移动后不同视角点云不能直接叠加 | 建立 `world → base_link` 并发布 world 点云 |
| 7. RViz 在 world 下长期运行卡死 | PCA 和 Marker 直接在 world 中发布 |
| 8. 单视角半径明显低估 | 引入多视角融合补充横截面信息 |
| 9. 固定时间自动累加导致 fused cloud 持续膨胀 | 不再按时间盲目累加，加入 novelty 判断和 ACCEPT / IGNORE / REJECT |
| 10. fused model 自己作为下一轮边界导致长度逐渐漂移 | 将参考模型和融合点云分离，用稳定参考主轴约束后续观测 |
| 11. 第一帧轴向范围固定过死，导致长度可能长期偏短 | 允许端点在连续多帧一致证据支持下逐步扩展 |
| 12. `world <- base_link` 偶发 future extrapolation | exact-time TF 短时等待 + 重试队列，不再直接丢帧，也不使用错误的 latest TF |

目前在线融合最核心的判断已经稳定为：

```text
有效的新表面
→ ACCEPT

重复表面
→ IGNORE

与当前枝条几何明显不一致
→ REJECT
```

---

## 11. 后续工作

**当前在线多视角融合已经完成，不再把“自动融合”列为 Step13 未完成项。**

Step13 目前剩余的主要工作已经转为工程收尾：

- 整理 `perception.launch.py`；
- 整理各节点终端日志，只保留真正有用的 `[TARGET] / [FUSION] / [MODEL] / [TF]` 信息；
- 整理 RViz 最终展示；
- 统一对后续规划提供 \(\{\mathbf p_0,\mathbf d,L,r\}\)；
- 最后检查 perception package 的目录、依赖、重复代码和临时调试内容；
- 完成最终 Step13 文档和 Stage C 冻结。

在完成当前仿真版本后，后续真正值得扩展的方向主要是：

- 从当前规则化圆柱目标推广到真实不规则枝条；
- 将“单根局部枝段”进一步扩展成多枝段结构；
- 根据模型置信度或信息增益决定 UAV 是否还需要换视角；
- 将最终几何模型直接提供给后续 UAV 接近、切割位姿和机械臂规划；
- 后续可以把当前红色 Mask 替换为 YOLO / VLM 等更真实的目标识别方法，但后面的 3D 几何链路仍然可以继续复用。

因此当前 Step13 的感知框架已经从：

```text
RGB 看见目标
```

推进到：

```text
RGB 判断目标
  ↓
LiDAR 获取三维点
  ↓
深度聚类去背景
  ↓
world 坐标统一
  ↓
在线多视角鲁棒融合
  ↓
{ p0, d, L, r }
```

后面可以直接向规划和真实树木场景继续延伸。

---

# 12. 整吧整吧发论文？

当前已经不再只是：

```text
深度聚类 + 手动多视角融合
```

而是形成了一条更完整的逻辑：

```text
RGB 目标识别
  ↓
LiDAR 三维筛选
  ↓
深度聚类去除背景穿透
  ↓
world 坐标统一
  ↓
单视角 PCA / 圆柱建模
  ↓
在线多视角增量融合
  ↓
信息有效性判断
  ├─ ACCEPT
  ├─ IGNORE
  └─ REJECT
  ↓
参考模型约束
  ↓
端点多帧确认与持续修正
  ↓
{ p0, d, L, r }
```

目前可以进一步整理成一个比较明确的研究方向：

> **面向 UAV 枝条切割任务的 RGB–LiDAR 在线鲁棒多视角三维建模方法**

其中现在已经具备的核心内容包括：

```text
深度聚类
+
多视角几何建模
+
在线增量融合
+
新视角信息有效性判断
+
模型约束与异常观测拒绝
+
模型持续修正
```

当前结果已经说明：

- 单视角长度较稳定，但半径容易明显低估；
- 多视角可以显著补充横截面信息；
- 简单“定时累加 + voxel”会造成严重模型漂移；
- 加入模型一致性和新信息判断后，同视角重复观测能够被 IGNORE；
- 错误或明显不一致的观测能够被 REJECT；
- 有效新视角能够被 ACCEPT 并逐步改善几何模型；
- 最新在线实验最终达到 \(L=0.3021\,\mathrm{m}\)、\(r=0.0371\,\mathrm{m}\)。

后续若要进一步提高论文完整性，可以继续加入：

```text
模型置信度 / 成熟度
  ↓
主动选择下一个最佳视角
  ↓
真实枝条实验
  ↓
切割任务闭环验证
```

也就是说，后面真正值得做的创新不再是“继续往 fused cloud 里堆更多点”，而是：

> **让 UAV 知道什么时候模型已经够好、什么时候还需要换视角，以及这些几何信息怎样真正服务后续切割任务。**