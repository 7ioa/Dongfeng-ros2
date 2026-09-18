# 东风沙盘三维模型 · 第一版

根据提供的 6 张场景参考照片和实测尺寸生成。整体采用实际沙盘尺寸，单位为米。道路、建筑和景观都是三维网格；可以旋转查看、导入 Blender，并作为 Gazebo 的静态实验场景。

## 1. 先查看模型

**双击 `preview.html`，用 Edge 或 Chrome 打开即可。** 文件包含全部模型数据和查看器，不需要联网，也不需要先安装 ROS 2、Gazebo 或 Blender。

- 左键拖动：旋转；滚轮：缩放；右键拖动：平移。
- “整体视角”：查看完整场景。
- “俯视布局”：核对道路和地块位置。
- “侧看高差”：查看前后路面高度；关闭建筑、树木、设施和围挡后更清楚。
- 顶部复选框可以控制显示内容；这些开关只影响预览，不修改导出的模型。

## 2. 文件怎么用

| 文件 / 文件夹 | 用途 |
|---|---|
| `preview.html` | 可离线打开的交互式三维预览 |
| `dongfeng_sandbox.glb` | 带材质的单文件 3D 模型，建议用它导入 Blender |
| `dongfeng_sandbox.obj` + `.mtl` | 通用网格和材质，二者放在同一个目录 |
| `models/dongfeng_sandbox/` | Gazebo 静态场景模型，含显示网格和碰撞网格 |
| `worlds/dongfeng.sdf` | Gazebo 场景，包括光源、物理系统和查看窗口配置 |
| `launch_gazebo.sh` | 在 Ubuntu 中设置资源路径并启动场景 |
| `scene_config.json` | 实测尺寸与本次采用的估计参数 |
| `front_markings.json` | AB 端至中央路口南侧的标线位置、斑马线间距等参数 |
| `b_buildings.json` | B 区楼体轮廓、位置、朝向、高度及组合高楼连接段的参数 |
| `b_lake.json` | B 区细长湖和右侧水道的曲线控制点、岸宽与表面高度 |
| `generate_scene.py` | 生成所有三维几何和仿真资源的 Python 源码 |
| `build_preview.py` | 根据生成的网格更新离线预览 |
| `validate_exports.py` | 独立读取导出文件，检查 GLB、OBJ 和道路碰撞网格 |
| `validation.json`、`export_validation.json` | 本次生成与导出检查结果 |
| `ab_height_check.png` | 本次 AB 端修正后的路面高度检查图；之后自行修改模型时需另行更新此图 |
| `front_markings_preview.png` | 本次前段标线的俯视核对图，直接读取模型网格绘制；为便于核对隐藏了建筑、树木和设施 |
| `front_markings_validation.json` | 本次标线修改范围的核对记录；之后重新生成模型不会自动刷新此记录 |
| `b_buildings_preview.png`、`b_buildings_rear.png`、`b_buildings_plan.png` | 更新后的 B 区建筑正面、背面与俯视预览，来自实际导出模型的浏览器渲染 |
| `b_buildings_validation.json` | 本次 B 区修改范围及占地核对记录，之后重新生成模型不会自动刷新此记录 |
| `b_lake_preview.png`、`b_lake_validation.json` | 两栋 L 形低楼与细长湖调整后的局部预览、修改范围及湖岸检查记录 |
| `import_to_blender.py` | 可选：在 Blender 中建立新场景并保存 `.blend` |

## 3. 在 Ubuntu 24 虚拟机里加载

以下步骤面向已经安装 **Gazebo Harmonic** 的环境。当前模型只包含沙盘，加载场景本身不需要机器人或 ROS 2 节点。

1. 将整个 `dongfeng_sandbox` 文件夹复制到虚拟机，例如 `~/dongfeng_sandbox`。不要只复制 SDF 文件。
2. 在终端运行：

```bash
cd ~/dongfeng_sandbox
bash launch_gazebo.sh
```

### VMware 虚拟机出现闪烁或卡死时

**当前用户已反馈下面的启动方式有效，建议在本项目的 VMware Ubuntu 虚拟机中优先使用。** 先关闭已有 Gazebo 窗口，再运行：

```bash
cd ~/dongfeng_sandbox
QT_QPA_PLATFORM=xcb bash launch_gazebo.sh --render-engine ogre
```

其中 `QT_QPA_PLATFORM=xcb` 指定 Qt 的 X11 显示后端，`--render-engine ogre` 切换为 Ogre 1 渲染引擎。这些设置只作用于本次启动；后续打开场景时可继续使用这条命令。

### 手动设置资源目录

也可以手动设置资源目录：

```bash
cd ~/dongfeng_sandbox
export GZ_SIM_RESOURCE_PATH="$PWD/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r worlds/dongfeng.sdf
```

如果终端提示 `gz: command not found`，说明当前终端还没有可用的 Gazebo 命令，需要检查安装及环境加载。

本场景按 Gazebo Harmonic 的 SDF 世界与静态模型结构组织，参考 [Gazebo 世界文件文档](https://gazebosim.org/docs/harmonic/sdf_worlds/) 和 [模型构建文档](https://gazebosim.org/docs/harmonic/building_robot/)。

## 4. 在 Blender 里继续修改

推荐通过“文件 → 导入 → glTF 2.0”选择 `dongfeng_sandbox.glb`。模型以米为单位，GLB 已包含坐标轴转换，不要再缩放 100 倍或 1000 倍。切换到材质预览即可查看颜色。

如果用 OBJ，导入时采用 **Y Forward、Z Up，缩放 1**，并保留旁边的 MTL。对象按功能和材质分组，例如建筑、道路、植被和标线；同一材质下的多栋建筑可能位于同一个网格对象，需要时可在编辑模式按松散部件分离。

可选的命令行转换：

```bash
blender --background --python import_to_blender.py
```

该脚本创建新场景，导入 OBJ，并将结果保存为模型文件夹中的 `dongfeng_sandbox.blend`。交付包目前提供 GLB 和 OBJ；`.blend` 需要在装有 Blender 的电脑上生成。该转换脚本尚未在本机运行。

## 5. 坐标和尺寸

正向视角：近端左侧为公园水池，近端右侧为高楼群，远端为圆形绿岛与灰色场地。

- 原点：**挡板内侧有效矩形的左近角**，高度取灰色场地基准。
- X 向右，范围 0～3.30 m；Y 向远处，范围 0～5.40 m；Z 向上。
- A 左近、B 右近、C 右远、D 左远。这里的 A～D 是方位标记。
- 外围挡板位于这块有效区域之外，因此整个模型的包围盒略大于 3.30 × 5.40 m。
- OBJ、SDF 与源码采用 Z 向上；GLB 按格式标准使用根节点完成 Y 向上的转换。

| 项目 | 第一版采用值 | 来源 |
|---|---:|---|
| 挡板内有效尺寸 | 左右 3.30 m，前后 5.40 m | 实测 |
| 外围单向路面宽度 | 0.30 m | 实测 |
| 中央道路两侧边线间距 | 0.62 m | 最新指定值，替代之前的 0.65 m |
| 中央白线宽度 | 0.005 m | 实测 |
| 四处弯道外侧路面半径 | 0.80 m | 实测 |
| 对应弯道内半径 | 0.50 m | 由 0.80−0.30 推导 |
| AB 端中间横向直路高度 | 0 m | 用户补充确认 |
| A/B 圆弧最高点 | 0.05 m | 实测；峰值位置近似放在圆弧中部 |
| C/D 远端最高路面 | 约 0.13 m | 实测 |
| 后方各侧直坡斜长 | 0.89 m | 实测，按斜面长度建模 |

外围道路的显示面与碰撞面采用相同高度，AB 中间直路的路面高度实际为 0 m。为避免显示闪烁，底层地面设在 −1 mm，标线等装饰表面略高于道路。外围道路网格宽度为 0.30 m；AB 前段外侧白线沿路面边缘布置，内侧保留地块路缘白线，开口处不额外画封闭内圈。没有把所有“白线到白线”的测量方式视为毫米级测绘结果。

### 已明确采用的近似

- 后方直坡先升高 6.5 cm，对应水平长度约 88.76 cm；随后圆弧继续升高 6.5 cm，到达后端 13 cm。该分配按用户授权近似。
- AB 端中间横向直路保持 0 m，不再整体抬升。A、B 两侧圆弧各在中部达到 5 cm，圆弧两端平滑回到相邻直路的 0 m；从 A 峰值经中间直路到 B 峰值呈现“高—低—高”。升降坡面位于圆弧内。峰值具体位置与平滑坡形为近似。
- 内部道路和地块基底恢复到统一的 0 m 基准；旧版前端整体 5 cm 平台及侧面直路前段的过渡坡已取消。建筑、树木和路灯继续按照各自离地高度放置。
- 中央交叉口大致位于 Y=2.10～2.72 m；圆形绿岛中心约为 (1.65, 3.98) m。
- 道路离内部边框约 4 cm；挡板、护栏、路缘、地块圆角与建筑高度均根据照片估计。
- 建筑外形、立面框架、公园路径、水池、树木、路灯、信号灯和门架以接近照片布局为目标。没有进行照片测量标定或点云重建。

**精度范围：** 已知尺寸约束写入模型；未测坐标和建筑外形为照片估计，尚不能给出它们相对于实物的数值误差。

## 6. 后续修改与重新生成

### AB 至中央路口的标线调整

- 中央左车道改为照片中的偏折箭头，右车道保留细长直行箭头，靠中央路口处增加直行与左右转向组合箭头。
- AB 横向道路增加直行与转向组合箭头，以及公园和建筑地块下方的四条横向白线。
- 调整中央道路近端横线与中心线的接续，去掉旧版多余的短线；中央路口南侧斑马线改为更细密的 24 条白条。
- 调整前段外围两侧横线，修正前段外侧白线的面朝向，使其从上方可见。白线继续随原路面高差起伏。

标线形状和位置按实物照片近似，尚未进行照片测量标定。只更新本段标线；建筑、信号灯、树木、路面高度、碰撞网格和后段标线保持原样。

本项目的“路面贴图”采用紧贴路面的白色网格绘制，因此修改 `front_markings.json` 可调整位置、尺寸和间距；箭头轮廓在 `generate_scene.py` 的 `foreground_arrow()` 中，前段布置在 `build_foreground_markings()` 中。`front_markings_preview.png` 是本次核对图，自行改动后不会随生成脚本自动更新；以重新生成的 `preview.html` 为准。

### B 区建筑重建

- 前端两栋低楼改为完整 L 形轮廓，使用连续的浅灰屋顶，调整长短翼的方向与比例。
- 在靠中央路口一侧补齐两栋相向开口的低楼。
- 左侧细高楼调整楼高与底座长度；右侧组合楼增加顶部连接段、内凹立面、底座及最高塔楼的屋顶围边。
- 高楼立面改为连续菱形网格，低楼保留横向楼层线，屋顶增加细密接缝。

尺寸与位置依照照片近似，未新增建筑实测数据。建筑重建时曾保留旧湖，并据此适配低楼；下述局部调整已进一步纠正低楼与湖的相对位置。建筑实体的碰撞网格同步更新。

今后可修改 `b_buildings.json`：`origin` 为平面位置，`outline` 为相对于该位置的建筑轮廓，`yaw` 为绕竖直轴旋转的弧度，`height` 为楼体高度；组合高楼用 `bounds` 指定矩形范围，用 `bottom` 指定相对于基座的起始高度。外观细节在 `generate_scene.py` 的 `b_volume()`、`b_facade()` 和 `build_b_district()` 中。三张 B 区 PNG 为本次渲染记录，不会随生成脚本自动刷新。

### 本次两栋 L 形低楼与湖的局部调整

- 保留左侧 L 形楼的位置，将右侧楼移到它旁边并摆正，两楼长翼并排，短翼分别向两侧伸出。保持原楼高和立面样式。
- 湖的主体移到低楼后方，改为更细长的轮廓，向右前方收窄并连接细水道；岸边使用窄灰色边缘。
- 高楼、后排开口低楼、地块边界、树木、路灯、道路、标线、原有步道及 A 区水池均保持原样。未更新 ZIP。

湖形由 `b_lake.json` 的连续三次贝塞尔曲线生成，每段 `curves` 中依次是两个控制点和终点，`start` 是第一段起点。`bank_width` 为灰色岸边宽度。这些曲线按照片近似，没有新增实测数据。

### 停车场出入口与道闸修正

原先生成脚本在灰色场地（停车场）前边界的两个端点处放了两个立柱，位置在 y=4.04，比场地前沿（y≈4.128）偏外 8.8 cm，正好卡在道路内侧护栏与场地前角之间约 5 cm 的空隙里；而且场地前边界由栏杆和树列连续铺满，没有开口，立柱上也没有横杆，因此起不到出入口放行的作用。

本次修正：

- 在停车场前边界两端各开一个车辆开口（左侧 x∈[0.39, 0.853]，右侧 x∈[2.447, 2.91]，各宽 0.463 m）。开口处的栏杆、立柱和白边线均断开，树列也让开。
- 两个立柱改为完整的道闸：只有一根立柱，立在场地前角（黄色基座）。**立杆与横杆是同一根杆**——竖直升到横杆高度后折 90° 变成悬挑横杆，拐角上方不再有竖杆；杆身白色带红带（立杆两段、横杆三段）。控制柜在立柱内侧。开口另一端不设立柱。两个道闸完全相同，不区分进口与出口。
- 原在 (0.65, 4.15) 和 (2.65, 4.15) 的两盏路灯移到开口靠内侧的边界上（x=0.90 / 2.40），避免立在车道上。

开口范围和道闸位置由 `generate_scene.py` 中的 `yard_front_y()`、`YARD_OPENINGS`、`YARD_GATE_LAMPS` 与 `yard_gate()` 统一控制。清除树木时仍会照常抽取随机数，因此其余 206 株树的叶片配色与位置与修改前完全一致。道路、建筑、地块、围挡、桥面、标线和碰撞网格均未改动。

### 重新生成

Python 生成程序只依赖 `numpy`：

```bash
python3 -m pip install numpy
python3 generate_scene.py
python3 validate_exports.py
python3 build_preview.py
```

尺寸参数在 `scene_config.json`，前段标线参数在 `front_markings.json`，B 区建筑参数在 `b_buildings.json`，B 区湖形在 `b_lake.json`；修改后执行上述三个脚本。其余建筑、地块细节与部分景观坐标写在 `generate_scene.py` 中。该生成器针对当前 3.3 × 5.4 m 沙盘编写；大幅改动总尺寸时，也需要调整源码中的建筑和道路位置。重新生成会覆盖同名导出文件，手工修改后的模型请另存。

## 7. 验证情况与导航实验接口

已完成：

- 浏览器整体、俯视和侧向预览检查，图层开关检查；检查时无浏览器错误日志。
- 外围 0.30 m 宽度和后方 0.89 m 斜长的几何检查。
- 已独立读取导出网格，确认 AB 中间直路为 0 m、两处圆弧峰值为 0.05 m，圆弧与直路连接处为 0 m，且显示路面与碰撞面一致。
- GLB 文件结构、数据范围与 OBJ 对应关系检查。
- 三角面索引和有限数值检查，导出视觉网格及碰撞网格无零面积三角面。
- 道路碰撞体闭合检查：无开放边、无非流形边、无方向不一致的相邻边。
- SDF XML 可解析，模型资源引用均存在。
- 前一次标线更新已核对：该次修改的 44 个非标线资源文件内容保持一致，Y>2.12 m 的 3172 个后段标线三角面保持一致；前段标线朝上，且已生成俯视核对图。
- 前一次 B 区建筑重建已核对：该次修改的 38 个受保护文件（包含原 ZIP）内容不变，其他区域建筑的 2898 个显示与碰撞三角面保持一致；历史记录见 `b_buildings_validation.json`。
- 本次低楼与湖调整核对 40 个受保护文件和 6 组混合网格中的保留三角面；湖岸无自交，低楼与湖的位置按 3 mm 间距检查，结果见 `b_lake_validation.json`。PNG 为浏览器预览，非 Gazebo 截图。

模型约 11.3 万个显示三角面、5.0 万个碰撞三角面，包含 206 株简化树木。静态道路、建筑、路缘、树干、挡板和部分设施设有碰撞几何；水面、标线、树冠和小型立面装饰主要用于外观显示。

**Gazebo 加载反馈：** 用户已在 Ubuntu 虚拟机中成功加载模型，并反馈使用 `QT_QPA_PLATFORM=xcb bash launch_gazebo.sh --render-engine ogre` 对缓解闪烁、卡死有效。此项为用户运行反馈；车辆通行、传感器和物理仿真验证尚未完成。

本次 B 区修订完成了导出检查和浏览器中的三视角核对，尚未在 Gazebo 中实际运行。

这个模型是后续小车仿真的三维环境。SLAM 的二维占据地图需要在加入小车、激光雷达和相应 ROS 2 节点后生成；本次没有预制扫描地图或导航结果。

## 8. 地图与四轮小车：键盘驾驶

在原有小车控制接口上，按五张实车照片重建黑色四轮车体、青色辐条轮毂、双层镂空板、前部弧形护框、电路板与线缆、五颗灰色标记球，以及前向摄像头、前置圆柱雷达和后部充电口。**本项目目录同时是 colcon 工作区**，机器人相关代码在 `src/` 下，与场景生成解耦。地图沿用同伴更新的版本，小车仍在运行时通过 Gazebo create 服务单独生成。

此次是外形与安装坐标的近似重建，尺寸依据现有约 20 cm 车长和照片比例。摄像头与雷达目前包含外壳、碰撞体和坐标系，尚不输出图像、点云或 `/scan`。

### 8.1 启动地图与小车

先关闭上一次 Gazebo 仿真。将整个 `dongfeng_sandbox` 文件夹拖到 Ubuntu 中，在该文件夹打开终端：

```bash
cd ~/dongfeng_sandbox
bash launch_car.sh
```

脚本自动加载 ROS 2、编译两个小车包、打开现有地图、生成小车并启动控制桥接。默认使用之前有效的 **xcb + Ogre** 设置。小车生成在 **AB 端横向直路中间**，车头朝地图 +X（从 A 朝 B）。不需要再另开 `launch_gazebo.sh`。

编译产物放在 `build_control/`、`install_control/`、`log_control/`，避免复用同伴电脑中的绝对路径；每次启动会刷新 CMake 缓存，文件夹移动后也可以重建。摄像头与雷达的数据功能不参与本次启动。

如果提示缺少依赖，在 Ubuntu 中安装一次：

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher ros-jazzy-rclpy \
  ros-jazzy-geometry-msgs python3-colcon-common-extensions
```

启动参数可以直接追加，例如无界面运行：

```bash
bash launch_car.sh headless:=true
```

常用 launch 参数：`x` `y` `z` `yaw`（默认 x=1.65、y=0.19、z=0.031 m、yaw=0）、`headless`、`render_engine`（默认 ogre）、`use_sim_time`（默认 true）、`world`、`models_path`、`command_timeout`（默认 0.4 秒）。雷达和摄像头所在一端就是车头。

`bash launch_gazebo.sh` 仍是仅查看静态地图的入口。小车无需写入地图网格，它由 launch 生成到同一个 Gazebo 世界中。若控制节点异常退出，整个启动流程会停止；若生成小车失败，会打印错误并退出。

### 8.2 键盘控制

另开一个终端：

```bash
cd ~/dongfeng_sandbox
bash keyboard_control.sh
```

保持键盘终端处于焦点，使用英文输入法，按住对应按键：

| 按键 | 动作 |
| --- | --- |
| W / I | 前进 |
| S / , | 后退 |
| A / J | 原地左转 |
| D / L | 原地右转 |
| U / O | 向前左转 / 向前右转 |
| 空格 / K | 发送停车指令 |
| + / - | 调高 / 调低速度，调整时先停车 |
| Q / Ctrl+C | 停车并退出键盘程序 |

默认前进速度 **0.10 m/s**，转速 **0.65 rad/s**；最高限制为 ±0.25 m/s、±1.2 rad/s。终端没有按键释放事件，因此用按键重复判断长按：**松键约 0.65 秒后发送零速**，车体随后按减速度停止；需要主动停车时按空格。若 Ubuntu 的按键重复被关闭，持续按住会变成短时移动；可在系统键盘设置中启用按键重复。方向键不支持，收到方向键序列时停车。

控制链路：`键盘 → ROS /cmd_vel → command_guard → ROS /cmd_vel_safe → ros_gz_bridge → Gazebo /cmd_vel → 四轮 DiffDrive`。控制节点限速、过滤非有限值，并在 **0.4 秒未收到新命令**时发送零速；使用单调时钟，因此暂停仿真时也能让过期命令失效。退出键盘程序后地图仍保持打开。

也可以继续用其他程序发布 ROS `/cmd_vel`，但应以至少 10 Hz 连续发布；单次命令会在超时后停车。不要同时运行多个键盘控制程序。

### 8.3 检查 /odom 与 TF

```bash
source /opt/ros/jazzy/setup.bash
source ~/dongfeng_sandbox/install_control/local_setup.bash
ros2 topic echo /cmd_vel_safe --once      # 发给 Gazebo 的限速后指令
ros2 topic echo /odom --once              # 位置、姿态、线速度与角速度
ros2 topic hz /odom                       # 应为 50 Hz
ros2 run tf2_ros tf2_echo odom base_link  # odom → base_link
ros2 run tf2_ros tf2_echo base_link lidar_link
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
ros2 topic echo /joint_states --once      # 四个车轮的角度与角速度
```

TF 树：`odom → base_link` 由 DiffDrive 经 gz `/tf` 桥接发布。四个转动车轮为 `wheel_left_link`、`wheel_right_link`、`wheel_left_front_link`、`wheel_right_front_link`，由 robot_state_publisher 根据 `/joint_states` 发布。固定坐标为 `base_link → lidar_link / camera_link → camera_optical_frame`；原来的前后球形支撑已替换为四轮结构。车体 +X 向前、+Y 向左、+Z 向上；相机光学坐标 +Z 向前、+X 向右、+Y 向下。

话题桥接在 `src/dongfeng_bringup/config/bridge.yaml`：`/clock`、`/cmd_vel_safe → Gazebo /cmd_vel`、`/odom`、`/tf`、`/joint_states`。`/odom` 的原点是小车启动时的位置，不能将它直接当作沙盘世界坐标。

### 8.4 小车参数

| 参数 | 当前近似值 |
| --- | --- |
| 车壳长、宽 | 0.200 × 0.140 m |
| 整体长、宽、离地总高（含轮毂、标记球） | 约 0.206 × 0.155 × 0.116 m |
| 轮半径、轮宽 | 0.028 m、0.018 m |
| 前后轴距、左右轮中心距 | 0.122 m、0.132 m |
| 总质量 | 1.502 kg，仿真假定值，未经称重 |
| 雷达坐标（相对 base_link） | (0.075, 0, 0.059) m，坐标原点位于外壳扫描环高度 |
| 摄像头坐标（相对 base_link） | (0.102, 0, 0.008) m，朝 +X |

`base_link` 位于轮轴平面，轮胎落地后距地面约 0.028 m。运动暂采用四轮差速近似，保持 `/cmd_vel`、`/odom` 等接口；没有根据真实底盘标定转向方式、摩擦和惯量。两侧各配置两个驱动关节，使用 Gazebo DiffDrive 的多关节配置（[官方接口说明](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html)）。碰撞使用简化车体和圆柱轮胎，细小外观零件不逐个参与碰撞。

**查看小车：** 用浏览器打开本目录的 `vehicle_preview.html`，可切换车头、侧面、车尾、俯视，也可拖动旋转。它独立于地图的 `preview.html`。`vehicle_preview/dongfeng_car.glb` 可导入 Blender 等建模软件查看整体外形；Gazebo 使用的是包内 OBJ 网格和 URDF。

**修改外形：** 编辑 `src/dongfeng_description/scripts/generate_vehicle.py`，其中顶部是主要尺寸，`build_body()`、`build_upper()`、`build_wheel()`、`build_sensors()` 分别构建车壳、上层结构、轮子和传感器。该生成器只处理小车，不调用地图生成脚本。

```bash
# 外形参数改动后执行；需要 Python 3 和 numpy。
python3 src/dongfeng_description/scripts/generate_vehicle.py
python3 src/dongfeng_description/scripts/build_vehicle_preview.py
python3 src/dongfeng_description/scripts/validate_vehicle.py
bash scripts/build_control.sh
source install_control/local_setup.bash
```

`urdf/dongfeng_car.urdf.xacro` 现在是生成后的普通数值 URDF，沿用原文件名以兼容启动流程；直接修改它会被下一次生成覆盖。脚本会同时更新 `meshes/` 与独立预览资源。关闭旧仿真再启动才能加载更新后的小车。

### 8.5 已知行为

- **动力学尚待验证**：四轮与双轮支撑不同，转弯会产生侧向摩擦；真实转弯半径、爬坡和碰撞行为需要在 Ubuntu Gazebo 中进一步验证和调整。四轮由同一个 DiffDrive 控制器驱动；前进、倒退及左右转向均有对称速度/加速度限制。
- **当前需要人工驾驶**：模型尚未接入传感器数据与避障，驶出道路或顶住路缘时需停止控制指令。
- 启动日志中 `kdl_parser: The root link base_link has an inertia specified in the URDF` 是无害提示（KDL 不支持带惯量的根链接），不影响仿真与 TF。

### 8.6 本阶段不包含

SLAM、Nav2、激光扫描、相机图像生成、视觉算法和传感器融合。摄像头和雷达外形及坐标已经完成，后续可在这些坐标系上接入仿真传感器。

此次已完成五方向浏览器预览、URDF/OBJ/GLB 静态检查、四轮坐标核对，以及 11 项键盘/超时/限速控制回归检查。出生位置覆盖范围内的 775 个地面采样点高度均为 0，初始轮胎距路面 3 mm，用于重力落地。地图与压缩包内容未改变。检查结果见 `vehicle_preview/validation_report.json`、`control_validation.json`。

可独立复查控制逻辑（无需启动 ROS）：

```bash
python3 src/dongfeng_bringup/test/test_control.py
```

当前 Windows 环境没有运行 ROS 2/Gazebo，实际驾驶效果仍需在 Ubuntu 虚拟机中启动验证。超时停车是针对 Gazebo DiffDrive 会保留最后速度指令的行为补充的（[官方实现](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/diff_drive/DiffDrive.cc)）。
