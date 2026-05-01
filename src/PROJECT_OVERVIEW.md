# LeRobot SO101 机械臂 ROS 2 仿真与控制项目

## 项目概览

本项目是一个基于 **ROS 2 Humble** 的机械臂仿真与控制系统，硬件平台为 **SO101 机械臂**（基于 [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) 开源模型修改而来）。项目运行环境为 **Ubuntu 22.04 + WSL2**，使用 **Gazebo Classic** 作为物理仿真引擎，集成 **MoveIt 2** 进行运动规划和轨迹执行，目标是实现 **草莓采摘（Strawberry Harvesting）** 的完整仿真任务。

仿真场景已从简单抓取方块的 demo **移植为草莓农业场景**（来自 [strawberry_gazebo_sim](https://github.com/) 项目），包含泥土地面、草莓种植床（raised bed）、以及程序化生成的草莓植株模型。

---

## 工作空间结构

```
lerobot_ws/src/
├── lerobot_description/     # 机械臂 URDF/Xacro 描述文件、网格、Gazebo 配置
├── lerobot_controller/      # ROS 2 Control 控制器配置与启动文件
├── lerobot_moveit/          # MoveIt 2 运动规划配置与启动文件
├── position_topic/          # 目标位姿发布/订阅节点、Gazebo 模型与场景（含草莓场景资产）
└── IFRA_LinkAttacher/       # 第三方 Gazebo 链接附着/分离插件（抓取用）
```

## 历史提交摘要

| 提交 | 说明 |
|------|------|
| `fc2d715` | 适配 ROS 2 Humble，Gazebo Harmonic → Classic 降级，修复 WSL2 兼容性 |
| `16372fd` | 添加机械臂移动控制 demo、抓取 demo（不可用） |
| `b99ccab` | 在 Gazebo 中添加待抓取目标与深度相机，实现基本系统搭建 |
| `cc931c3` | 修改 README 文档 |

### 未提交的重要改动（当前工作区状态）

| 文件 | 改动 |
|------|------|
| `lerobot_description/launch/so101_gazebo.launch.py` | World 文件切为 `pick_place_strawberry.world`；添加 `GAZEBO_MODEL_PATH` |
| `lerobot_description/urdf/so101_base.xacro` | `base_joint` z 偏移设为 `0.175`（机械臂底座放在 bed 顶面） |
| `lerobot_description/urdf/so101.urdf.xacro` | 引入 `so101_camera_in_hand.xacro` 并挂载到 gripper |
| `lerobot_description/urdf/so101_camera_in_hand.xacro` | **新文件**：手眼深度相机宏（`ee_camera_` 命名前缀） |
| `position_topic/setup.py` | 改为 `os.walk()` 递归安装 models/ 和 worlds/ 子目录 |
| `position_topic/models/bed/` | **新目录**：草莓种植床模型（11 个文件：SDF + COLLADA 网格 + 纹理） |
| `position_topic/models/dirt_plane/` | **新目录**：泥土地面模型（含 ~60MB 纹理） |
| `position_topic/models/random_strawberry_plant/` | **新目录**：程序化草莓植株（124 个文件：SDF + ERB 模板 + 网格 + 纹理） |
| `position_topic/worlds/pick_place_strawberry.world` | **新文件**：合并草莓场景 + LinkAttacher 的世界文件 |

---

## 各包详细说明

### 1. lerobot_description — 机械臂描述包

**路径**: `lerobot_description/`

**用途**: 提供 SO101 机械臂的完整 URDF/Xacro 模型描述，包括视觉网格、碰撞几何、物理惯性参数、ros2_control 接口定义、Gazebo 插件配置，以及固定在末端的深度相机。

**文件结构**:

```
lerobot_description/
├── CMakeLists.txt                    # 安装 launch/urdf/meshes/rviz 到 share 目录
├── package.xml                       # 依赖: robot_state_publisher, rviz2, xacro 等
├── urdf/
│   ├── so101.urdf.xacro             # 主入口——引用下面所有 xacro 文件，挂载手眼相机
│   ├── so101_base.xacro             # 本体结构: 6 个 link + 6 个关节 + 传动定义
│   │                                #   base_joint z=0.175 (放在 bed 顶面)
│   ├── so101_gazebo.xacro           # Gazebo 插件: libgazebo_ros2_control.so
│   ├── so101_ros2_control.xacro     # ros2_control 硬件接口: GazeboSystem, 6 个位置控制关节
│   └── so101_camera_in_hand.xacro   # 手眼相机宏: 固定在 gripper 上的深度相机 (640x480@30Hz)
├── meshes/so101/                     # 13 个 STL 网格文件（3D 打印件 + STS3215 舵机）
├── launch/
│   ├── so101_display.launch.py      # RViz 可视化 (无仿真)
│   └── so101_gazebo.launch.py       # Gazebo Classic 仿真启动: 加载草莓世界 + 生成机器人/方块/相机
└── rviz/display.rviz                 # RViz 配置文件
```

**运动学链条**:

```
world (原点)
  └── base_joint (fixed, xyz="0 0 0.175")
        └── base
              └── joint 1 (revolute, ±1.92 rad) → shoulder
                    └── joint 2 (revolute, ±1.75 rad) → upper_arm
                          └── joint 3 (revolute, -1.75~+1.57 rad) → lower_arm
                                └── joint 4 (revolute, ±1.66 rad) → wrist
                                      └── joint 5 (revolute, ±2.79 rad) → gripper
                                            ├── joint 6 (revolute, -0.17~+1.75 rad) → jaw
                                            └── ee_camera_joint (fixed) → ee_camera_link
                                                  └── ee_camera_optical_joint (fixed) → ee_camera_optical_link
```

**关键设计点**:
- `base_joint` z=0.175：将机械臂底座抬高到 bed 碰撞体顶面以上，视觉上坐落在 raised bed 上
- 所有关节使用 `PositionJointInterface` + `SimpleTransmission`
- 手眼相机使用 `ee_camera_` 前缀命名（避免与外部 Gemini 335 相机的 `camera_link` 冲突）

**关节列表**:

| 序号 | 关节名 | 父连杆 | 子连杆 | 限位 (rad) |
|------|--------|---------|---------|-------------|
| 1 | `1` | base | shoulder | ±1.91986 |
| 2 | `2` | shoulder | upper_arm | ±1.74533 |
| 3 | `3` | upper_arm | lower_arm | -1.74533 ~ +1.5708 |
| 4 | `4` | lower_arm | wrist | ±1.65806 |
| 5 | `5` | wrist | gripper | ±2.79253 |
| 6 | `6` | gripper | jaw | -0.174533 ~ +1.74533 |

---

### 2. lerobot_controller — 控制器配置包

**路径**: `lerobot_controller/`

**控制器配置** (`so101_controllers.yaml`):

| 控制器名 | 类型 | 控制关节 | 特点 |
|----------|------|----------|------|
| `joint_state_broadcaster` | JointStateBroadcaster | 全部 6 个 | 广播关节状态 |
| `arm_controller` | JointTrajectoryController | 1-5 | 开环控制，open_loop_control=true |
| `gripper_controller` | JointTrajectoryController | 6 | 开环控制，open_loop_control=true |

- `update_rate`: 10 Hz
- 启动顺序: joint_state_broadcaster → arm_controller → gripper_controller（通过 `RegisterEventHandler` 保证顺序依赖）
- 双模式: `is_sim=True`（Gazebo 接管 ros2_control） / `is_sim=False`（实物模式）

---

### 3. lerobot_moveit — MoveIt 2 运动规划包

**路径**: `lerobot_moveit/`

**关键配置**:

| 配置项 | 值 | 说明 |
|--------|-----|------|
| 运动学插件 | `KDLKinematicsPlugin` | 数值 IK 求解 |
| `position_only_ik` | `True` | 只求解末端位置，忽略姿态——适用于初始抓取引导 |
| 搜索精度 | 0.005m | |
| 超时 | 0.1s | |
| 速度缩放 | 0.1 (config) / 0.3 (subscriber) | 保守值，安全起步 |
| 规划组 | `arm` (关节 1-5 + base_joint), `gripper` (关节 6) | |
| 控制器桥接 | `arm_controller` / `gripper_controller` → `FollowJointTrajectory` | |

**碰撞禁用对** (so101.srdf): base-shoulder, base-lower_arm, shoulder-upper_arm, shoulder-lower_arm, lower_arm-upper_arm, lower_arm-gripper, lower_arm-wrist, wrist-gripper, wrist-jaw, gripper-jaw。**注意**: 未加入与草莓场景模型（bed/plant）的碰撞禁用。

---

### 4. position_topic — 目标位姿与草莓场景集成包

**路径**: `position_topic/`

**用途**: 项目核心集成包。包含目标位姿发布/订阅节点、Gazebo 草莓场景全套模型资产、世界文件、以及总启动文件。

**文件结构**:

```
position_topic/
├── setup.py                          # 递归安装 models/ worlds/ launch/；注册 2 个入口点
├── position_topic/
│   ├── position_publisher.py         # 目标位置发布节点 (PoseStamped @ 1Hz)
│   └── position_subscriber.py        # 目标位姿订阅→MoveIt 规划执行
├── launch/
│   └── move_demo.launch.py           # 总启动器
├── models/
│   ├── target_box.sdf                # 红色抓取目标: 3cm³ 立方体, 0.05kg
│   ├── camera.sdf                    # 外部深度相机: Gemini 335, 640x480@30Hz
│   ├── bed/                          # 草莓种植床: COLLADA 网格, 碰撞体 2×3.2×0.35m
│   ├── dirt_plane/                   # 泥土地面: 100×100m 平面, OGRE 材质纹理
│   └── random_strawberry_plant/      # 程序化草莓植株: ~120 片叶子 + ~10 个果实 + 茎杆
└── worlds/
    ├── pick_place_attacher.world     # 原始世界 (ground_plane + sun + LinkAttacher)
    └── pick_place_strawberry.world   # 草莓世界: dirt_plane + bed + plant + LinkAttacher
```

#### 草莓场景坐标系

| 对象 | pose (x, y, z) | 说明 |
|------|----------------|------|
| `dirt_plane` | `(0, 0, 0)` | 泥土地面，z=0 |
| `bed` | `(0, 0.5737, -0.25)` | 种植床，碰撞体 0.35m 高 → 顶面在 z≈-0.075 |
| `random_strawberry_plant` | `(0.25, 0.25, 0.20)` | 植株根部，果实距地面约 z≈0.21-0.22 |
| SO101 base_joint | `(0, 0, 0.175)` | 机械臂底座在 bed 上方 |
| `target_box` | `(0.2, 0, 0.015)` | 抓取用目标方块（保留） |
| Gemini 335 相机 | `(0.3, 0, 0.8)` | 外部深度相机，俯视 |

#### 节点详解

##### position_publisher
- **Topic**: `/target_pose` (PoseStamped)
- **默认位置**: `[0.30, 0.0, 0.25]`
- **频率**: 1Hz（可配）
- **姿态**: 固定为单位四元数（下游不使用）

##### position_subscriber
- **订阅**: `/target_pose` → Action Client `/move_action` (MoveGroup)
- **约束**: 仅位置约束（球形区域，半径 0.01m），无姿态约束
- **参数**: velocity_scaling=0.3, accel_scaling=0.3, planning_attempts=10, planning_time=5.0s

#### 草莓模型资产说明

| 模型 | 来源 | 内容 | 文件数 |
|------|------|------|--------|
| `bed` | strawberry_gazebo_sim | 塑料覆膜 raised bed，COLLADA 网格 + 纹理 | 11 |
| `dirt_plane` | strawberry_gazebo_sim | 100×100m 泥土平面，OGRE 材质（~60MB 纹理） | 5 |
| `random_strawberry_plant` | strawberry_gazebo_sim | ERB 模板程序化生成，含 5758 行 model.sdf | 124 |

`random_strawberry_plant` 模型使用 ERB (Embedded Ruby) 模板 (`model.rsdf`) 程序化生成，当前预生成的 `model.sdf` 包含约 10 个果实和约 120 片叶子。可通过 Ruby + rubystats gem 重新生成不同配置。

---

### 5. IFRA_LinkAttacher — 第三方抓取插件

**路径**: `IFRA_LinkAttacher/`

**来源**: [IFRA-Cranfield/IFRA_LinkAttacher](https://github.com/IFRA-Cranfield/IFRA_LinkAttacher) (Cranfield University, UK, Apache-2.0)

**两个子包**:

| 子包 | 说明 |
|------|------|
| `linkattacher_msgs` | 定义 `AttachLink.srv` 和 `DetachLink.srv` 服务类型 |
| `ros2_linkattacher` | C++ Gazebo World Plugin `libgazebo_link_attacher.so` |

**使用方式**:

```bash
# 附着物体到末端
ros2 service call /ATTACHLINK linkattacher_msgs/srv/AttachLink \
  "{model1_name: 'so101', link1_name: 'gripper', model2_name: 'target_box', link2_name: 'box_link'}"

# 释放物体
ros2 service call /DETACHLINK linkattacher_msgs/srv/DetachLink \
  "{model1_name: 'so101', link1_name: 'gripper', model2_name: 'target_box', link2_name: 'box_link'}"
```

---

## 系统启动流程

### 完整仿真启动

```bash
ros2 launch position_topic move_demo.launch.py
```

启动顺序:

1. **Gazebo Classic** (`so101_gazebo.launch.py`)
   - 设置 `GAZEBO_MODEL_PATH` 包含 position_topic/models
   - 启动 Gazebo 服务端 + 客户端
   - 加载 `pick_place_strawberry.world`（含 dirt_plane、bed、plant、LinkAttacher 插件）
   - 生成 SO101 机器人实体
   - 生成 target_box
   - 生成 Gemini 335 外部深度相机
   - 发布 world→camera_link 和 camera_link→camera_depth_optical_frame 的静态 TF

2. **ROS 2 Control** (`so101_controller.launch.py`)
   - 启动 ros2_control_node
   - 按序激活: joint_state_broadcaster → arm_controller → gripper_controller

3. **MoveIt 2** (`so101_moveit.launch.py`)
   - 启动 move_group
   - 启动 RViz

4. **position_subscriber** 节点
   - 等待 `/target_pose` → MoveIt 规划执行

### 仅 RViz 可视化（无仿真）

```bash
ros2 launch lerobot_description so101_display.launch.py
```

### 发布目标位置

```bash
ros2 run position_topic position_publisher --ros-args -p position:="[0.25, 0.05, 0.20]"
```

---

## 构建

```bash
cd ~/lerobot_ws
colcon build --symlink-install --packages-select lerobot_description position_topic lerobot_controller lerobot_moveit linkattacher_msgs ros2_linkattacher
```

---

## 技术栈

| 组件 | 版本/类型 |
|------|-----------|
| OS | Ubuntu 22.04 (WSL2) |
| ROS 2 | Humble |
| 仿真引擎 | Gazebo Classic (11.x) |
| 运动规划 | MoveIt 2 + KDL Kinematics |
| 控制器 | ros2_control (JointTrajectoryController) |
| 语言 | Python (position_topic, launch), C++ (LinkAttacher) |
| 构建系统 | ament_cmake / ament_python |
| 3D 格式 | URDF/Xacro, SDF, COLLADA (.dae), STL |

---

## 当前状态与已知问题

### 已完成
- [x] SO101 机械臂 URDF 建模与 Gazebo 仿真
- [x] ros2_control 控制器配置与关节控制
- [x] MoveIt 2 运动规划集成
- [x] IFRA LinkAttacher 抓取插件集成
- [x] 草莓农业场景移植（bed + dirt_plane + random_strawberry_plant）
- [x] GAZEBO_MODEL_PATH 自动注入
- [x] 手眼相机 URDF 宏定义（`ee_camera_` 命名前缀）
- [x] `position_only_ik` 引导式抓取规划

### 待完成
- [ ] **抓取全流程闭环**: 靠近→附着→抬起→移动→放置→释放 的完整自动化
- [ ] **vlm_bridge 节点**: 订阅相机图像/深度信息，为 VLM 推理提供数据通道
- [ ] **2D→3D 坐标映射**: 像素坐标 + 深度 → 相机坐标系 → world 坐标系（TF2 变换）
- [ ] **手眼相机 Gazebo 集成**: 虽然 URDF 已定义，但 launch 文件中未路由 `so101/camera/*` 话题

### 已知 Bug
- **手眼相机 frame_name 错误**: `so101_camera_in_hand.xacro` 第 67 行 `<frame_name>camera_optical_link</frame_name>` 应为 `ee_camera_optical_link`
- **碰撞禁用不完整**: MoveIt SRDF 中未禁用机械臂与草莓场景模型（bed/plant/dirt）之间的碰撞检测
- **开环控制**: `arm_controller` 和 `gripper_controller` 使用 `open_loop_control: true`，无反馈校正
- **速度缩放保守**: config 端 0.1，subscriber 端 0.3

### 坐标系注意事项
- 机械臂是 **fixed-base** 机器人（`base_joint` 是 fixed 类型），直接锚定在 world 坐标系上，不依赖物理支撑
- 调整机械臂位置时 **只能改 `base_joint` 的 xyz 偏移**，不能通过 spawn_entity 的 -x/-y/-z 参数
- 草莓植株的 `model.rsdf` 是 ERB 模板，如需重新生成不同配置需要 Ruby + rubystats gem
