# Cleanup manifest

本快照基于用户上传的 `UAV_lumberjack.zip` 整理，仅做工程结构清理，不改 Step13 已冻结的感知 / TF / 多视角融合 / TARGET / BranchModel 算法。

主要整理：

- `sim/step13_branch_perception` → `sim/gazebo`
- 删除 Step1~12 重复仿真工程副本；历史文档转入 `docs/archive/legacy_steps`
- 5 个演示视频集中到 `media/videos`，未删除视频
- 当前仿真 launch → `uav_lumberjack_sim.launch.py`
- 当前感知 launch → `perception.launch.py`
- 删除旧 MVP / Step7 / V2 / Step11 / Step12 launch
- 删除 `arm_j1_test.cpp` 与 `arm_controller_step5_backup.cpp`，历史说明归档
- 删除 `build/install/log`, Python cache, `.vscode`
- `.git` 不打包，GitHub 继续承担历史版本管理
- 根目录资料归入 `docs/` / `media/`
