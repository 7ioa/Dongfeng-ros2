# 外围自主巡航 Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** 在现有沙盘上完成一次含视觉交通灯停车的外围闭环巡航。

**Architecture:** 传感器输出经 ros_gz 桥接；独立视觉模块输出车道和灯色观测；路线跟踪、停车状态机和命令仲裁组成控制链。Gazebo 真值只进入验收工具。

**Tech Stack:** ROS 2 Jazzy、Gazebo Harmonic、Python、OpenCV、NumPy。

**Spec:** `docs/superpowers/specs/2026-09-21-autonomous-perimeter-design.md`

## Global Constraints

- 保留现有地图道路、建筑及高差；只改变交通灯所需的独立灯头。
- 从 `(1.65, 0.19)` 朝 +X 出发，经 B/C/D/A 返回起点；完成后保持停车。
- 驾驶不消费交通灯相位真值和 Gazebo 位姿真值。
- 默认速度 0.10 m/s，自动最大速度 0.15 m/s；命令继续经过 command_guard。
- 持久修改同步生成器；截图放 `check_screenshots/`，验收报告放 `reports/autonomy/`。

## Review Focus

1. 相机断流但算法仍发布旧结果：源时间戳必须触发停车。
2. 路线起点附近：不得刚启动就判完成，必须累计一整圈。
3. 颜色相似背景或错误方向灯：不能触发通行。
4. 人工停车：自动命令不得随即覆盖，需重新启用。
5. 坡道和弯道：雷达地面回波与图像投影变化不可导致撞障或长时间盲行。

## Task 1: 真实传感器与场景接口

Files: 修改 `generate_vehicle.py`、生成 URDF、`bridge.yaml`、`simulation.launch.py`；新增 `src/dongfeng_autonomy/` 包。

接口：`/camera/image_raw` Image、`/camera/camera_info` CameraInfo、`/scan` LaserScan、`/imu/data` Imu、现有 `/odom`。

- [ ] 增加 XML 配置测试，断言传感器名、话题、相机光学 frame 和 IMU 插件。
- [ ] 运行 `python3 -m unittest discover -s src/dongfeng_autonomy/test -v`，先观察缺失配置失败。
- [ ] 在生成器追加 camera/gpu_lidar/imu，并桥接；服务器渲染参数与 GUI 参数分离。
- [ ] 重建并运行无 GUI Gazebo，实际读取图像和扫描；验证镜头无遮挡与扫描非全空。

## Task 2: 路线和驾驶核心

Files: `dongfeng_autonomy/route.py`、`control.py`、`test/test_core.py`。

接口：`Route.project(x,y,previous)` 输出进度和横向误差；`Route.target(s)` 输出目标点；`Driver.step(pose, observation, now)` 输出线速度、角速度和状态。

- [ ] 测试闭环进度、四角顺序、初始不完成、里程计初始对齐、坏数值停车、过期观测、红黄绿状态和人工锁止。
- [ ] 运行测试，确认功能缺失导致失败。
- [ ] 生成与外围道路一致的圆弧/直线路径，实现前视跟踪、速度限制、灯前减速及一圈锁存。
- [ ] 测试在离散运动模型中完成一圈，并保持完成状态。

```python
assert Driver(route).state != 'COMPLETE'
# 颜色观测有效且连续绿灯满足门限，才可离开 WAIT_SIGNAL。
# 任何必需传感器收到时间超过 0.5 秒，输出必须为零。
```

## Task 3: 可观察的动态灯和视觉

Files: `signals.py`、`vision.py`、场景生成器、`test/test_vision.py`。

接口：灯配置含 ID、世界位置、朝向、停车弧长；`detect_light(image, roi)` 输出颜色/可信度；`detect_lane(image)` 输出偏差/可信度。

- [ ] 先写合成图像测试：红黄绿灯、黑图、植物绿背景、多个灯和错误 ROI。
- [ ] 独立灯头实体替换合并静态灯泡，控制真实场景中的红黄绿视觉状态。
- [ ] 颜色分割、灯头几何验证、连续帧确认及车道拟合。
- [ ] 在实际相机图像上验证三种灯色，保存调试帧。

## Task 4: ROS 集成和操作入口

Files: `autonomy_node.py`、`signal_node.py`、`arbiter_node.py`、launch、配置、构建脚本及 `launch_autonomy.sh`。

接口：自动命令 `/cmd_vel_auto`、手动命令 `/cmd_vel_manual`、仲裁唯一输出 `/cmd_vel`；`/autonomy/enable` SetBool 显式重新启用；`/autonomy/status` String JSON。

- [ ] 先测试过期源时间戳、时钟回退、急停后自动命令仍到达等条件。
- [ ] 感知节点和驾驶节点以墙钟 watchdog 处理断流；状态中包含观测年龄、停车原因和进度。
- [ ] 参数化所有阈值和相位；增加 ROS 包依赖和统一启动入口。
- [ ] 编译全包，验证手动入口仍可用，自动链路无重复速度发布者。

## Task 5: 物理验收、独立审查和使用说明

Files: `scripts/autonomy/validate_run.py`、`reports/autonomy/`、README。

- [ ] 实际运行完整外围路线，采集真值仅用于评估，检查道路边界、碰撞、四角顺序和回起点。
- [ ] 强制红转绿及改变初始相位，记录图像观测和实际停车。
- [ ] 验证障碍物、相机/雷达断流、人工接管和暂停恢复。
- [ ] 运行所有单元测试及原 11 项控制回归，记录实际结果和未验证项。
- [ ] 独立代码审查，修复重要发现，补回归测试。
- [ ] 更新 README 的依赖、启动、接管、调试与验收复现命令。

## 执行记录

用户已批准设计并明确要求继续完成开发；采用本会话直接实施，沿用交接文档中的当前目录开发约定。工作在当前目录的功能分支，不额外创建工作树；不推送远程。
