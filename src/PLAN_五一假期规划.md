# 五一假期项目冲刺计划

**总目标**：5天内跑通 "相机图像 → 目标3D坐标 → MoveIt规划 → 机械臂抓取" 全链路（VLM部分先用硬编码像素坐标替代）。

**日期**：2026年5月1日 – 5月5日  
**每日时段**：下午 14:00–17:00（固定，不拖到晚上）

---

## 前置检查清单（4月30日晚，15分钟）

```bash
# 确认当前代码能正常编译
cd ~/lerobot_ws
colcon build --symlink-install --packages-select lerobot_description position_topic lerobot_controller lerobot_moveit

# 确认 move_demo 能正常启动（启动后 Ctrl+C 关掉即可，只看有没有报错）
ros2 launch position_topic move_demo.launch.py
```

如果编译报错或启动报错，先修好再进 Day 1。

---

## Day 1（5月1日）：修复 TF 冲突 + 接入 eye-in-hand 相机

**目标**：eye-in-hand 相机在 Gazebo 中正常工作，图像 topic 有数据。

**预计耗时**：1.5–2 小时

### 背景

`so101_camera_in_hand.xacro` 已经写好并包含在 URDF 中，相机固定在 `gripper` 末端。但它定义的 link 叫 `camera_link` / `camera_optical_link`，和外部 Gemini 335 相机（`camera.sdf`）的 link 重名，TF 树会发生冲突。

### 操作步骤

#### Step 1：重命名 eye-in-hand 相机 link（避免 TF 冲突）

编辑 `lerobot_description/urdf/so101_camera_in_hand.xacro`：
- `camera_link` → `ee_camera_link`
- `camera_optical_link` → `ee_camera_optical_link`
- `camera_joint` → `ee_camera_joint`
- `camera_optical_joint` → `ee_camera_optical_joint`
- 插件 `<frame_name>` 改为 `ee_camera_optical_link`

编辑 `lerobot_description/urdf/so101.urdf.xacro`：
- 确认 `xacro:so101_camera_in_hand` 调用存在，parent="gripper"

#### Step 2：修改 launch 文件，TF 发布指向新 frame 名

编辑 `lerobot_description/launch/so101_gazebo.launch.py`：
- 外部相机（Gemini 335）保持不变
- 追加发布 `ee_camera_optical_link` 的 TF（或直接依赖 robot_state_publisher 从 URDF 生成）

> 实际上 eye-in-hand 相机的 TF 会由 `robot_state_publisher` 从 URDF 自动发布（`gripper → ee_camera_link → ee_camera_optical_link`），不需要手动写 static_transform_publisher。只要 link 名不冲突就行。

#### Step 3：验证

```bash
cd ~/lerobot_ws
colcon build --symlink-install --packages-select lerobot_description
ros2 launch lerobot_description so101_gazebo.launch.py
```

在另一个终端：

```bash
# 1. 检查 TF 树，确认 ee_camera_optical_link 存在
ros2 run tf2_tools view_frames
# 或
ros2 run tf2_ros tf2_echo base ee_camera_optical_link

# 2. 检查图像 topic
ros2 topic list | grep -i camera
# 预期看到：
#   /so101/camera/depth/image_raw
#   /so101/camera/color/image_raw
#   /so101/camera/depth/camera_info
#   /camera/depth/image_raw        (外部 Gemini 335)
#   /camera/color/image_raw

# 3. 查看图像是否有数据
ros2 run rqt_image_view rqt_image_view
# 选择 /so101/camera/color/image_raw，确认能看到画面
# 注意：因为相机装在 gripper 上，初始位置可能对着桌子/地面，有画面就OK
```

### 完成标准

- [ ] `ros2 topic echo /so101/camera/depth/camera_info` 能打印出相机内参
- [ ] `rqt_image_view` 能看到 eye-in-hand 相机画面
- [ ] TF 树无冲突报错

---

## Day 2（5月2日）：创建 vlm_bridge 节点框架

**目标**：新建一个 ROS 2 节点，订阅相机图像和深度图，打印图像尺寸。

**预计耗时**：2–2.5 小时

### Step 1：创建包结构

```bash
cd ~/lerobot_ws/src/position_topic/position_topic
touch vlm_bridge.py
chmod +x vlm_bridge.py
```

### Step 2：写节点骨架

`vlm_bridge.py` 初始内容：

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped


class VlmBridge(Node):
    def __init__(self):
        super().__init__("vlm_bridge")

        # 订阅 eye-in-hand 相机
        self.rgb_sub = self.create_subscription(
            Image, "/so101/camera/color/image_raw", self.rgb_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, "/so101/camera/depth/image_raw", self.depth_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/so101/camera/depth/camera_info", self.camera_info_callback, 10
        )

        # 发布目标位姿（给 position_subscriber 消费）
        self.target_pub = self.create_publisher(PoseStamped, "/target_pose", 10)

        self.latest_rgb = None
        self.latest_depth = None
        self.latest_camera_info = None

        self.get_logger().info("VLM Bridge 节点已启动")

    def rgb_callback(self, msg):
        if self.latest_rgb is None:
            self.get_logger().info(f"收到RGB图像: {msg.width}x{msg.height}, encoding={msg.encoding}")
        self.latest_rgb = msg

    def depth_callback(self, msg):
        if self.latest_depth is None:
            self.get_logger().info(f"收到深度图: {msg.width}x{msg.height}, encoding={msg.encoding}")
        self.latest_depth = msg

    def camera_info_callback(self, msg):
        if self.latest_camera_info is None:
            self.get_logger().info(
                f"收到相机内参: fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}, "
                f"cx={msg.k[2]:.1f}, cy={msg.k[5]:.1f}"
            )
        self.latest_camera_info = msg


def main():
    rclpy.init()
    node = VlmBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Step 3：注册入口点

编辑 `position_topic/setup.py`，在 `console_scripts` 中添加：

```python
"vlm_bridge": "position_topic.vlm_bridge:main",
```

### Step 4：编译 + 验证

```bash
cd ~/lerobot_ws
colcon build --symlink-install --packages-select position_topic
ros2 launch lerobot_description so101_gazebo.launch.py  # 终端1
ros2 run position_topic vlm_bridge                       # 终端2
```

预期终端2输出：
```
收到RGB图像: 640x480, encoding=rgb8
收到深度图: 640x480, encoding=...
收到相机内参: fx=..., fy=..., cx=..., cy=...
```

### 完成标准

- [ ] 三个订阅都收到数据，终端打印正常
- [ ] `ros2 topic list | grep target_pose` 确认话题存在

---

## Day 3（5月3日）：2D像素 → 3D坐标映射

**目标**：硬编码一个像素坐标，转换为 base 坐标系下的 3D 点，发布到 `/target_pose`。

**预计耗时**：2.5–3 小时

### 核心逻辑

```
像素 (u, v) + 深度值 d
  → 相机内参反投影 → 相机坐标系 3D 点 (x_c, y_c, z_c)
  → TF2 变换 → base 坐标系 3D 点 (x_b, y_b, z_b)
  → 包装为 PoseStamped → 发布到 /target_pose
```

### Step 1：添加反投影 + TF 变换逻辑

在 `vlm_bridge.py` 中添加以下方法：

```python
import numpy as np
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Point, Pose, Quaternion

class VlmBridge(Node):
    def __init__(self):
        # ... 前面的订阅代码 ...
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 定时器：每秒尝试做一次 2D→3D 映射
        self.timer = self.create_timer(1.0, self.process)

    def process(self):
        if self.latest_depth is None or self.latest_camera_info is None:
            return
        
        # ---- 硬编码目标像素（Day 4 会替换为 VLM API 输出）----
        u, v = 320, 240  # 图像中心点，先验证链路
        
        # 1. 从深度图取深度值
        depth = self.get_depth_at(u, v)
        if depth is None or depth <= 0:
            self.get_logger().warn(f"像素({u},{v})深度无效")
            return
        
        # 2. 反投影到相机坐标系
        fx = self.latest_camera_info.k[0]
        fy = self.latest_camera_info.k[4]
        cx = self.latest_camera_info.k[2]
        cy = self.latest_camera_info.k[5]
        
        x_c = (u - cx) * depth / fx
        y_c = (v - cy) * depth / fy
        z_c = depth
        
        # 3. TF 变换到 base 坐标系
        try:
            transform = self.tf_buffer.lookup_transform(
                "base", "ee_camera_optical_link", rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"TF查询失败: {e}")
            return
        
        # 简单变换（用四元数旋转 + 平移）
        t = transform.transform.translation
        q = transform.transform.rotation
        # 用 tf_transformations 或手写四元数旋转
        point_base = self.transform_point(x_c, y_c, z_c, t, q)
        
        # 4. 发布到 /target_pose
        self.publish_target(point_base)

    def get_depth_at(self, u, v):
        """从深度图中读取像素(u,v)的深度值（米）"""
        # depth 编码通常是 16UC1（毫米）或 32FC1（米）
        # Gazebo 的 libgazebo_ros_camera 输出 32FC1
        import numpy as np
        data = np.frombuffer(self.latest_depth.data, dtype=np.float32)
        data = data.reshape(self.latest_depth.height, self.latest_depth.width)
        return float(data[v, u])
    
    def transform_point(self, x, y, z, translation, rotation):
        """用四元数旋转3D点，再加平移"""
        # 简化版：直接用 tf_transformations
        from tf_transformations import quaternion_matrix
        import numpy as np
        q = [rotation.x, rotation.y, rotation.z, rotation.w]
        R = quaternion_matrix(q)[:3, :3]
        p_cam = np.array([x, y, z])
        p_base = R @ p_cam + np.array([translation.x, translation.y, translation.z])
        return p_base
    
    def publish_target(self, point_base):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base"
        msg.pose.position.x = float(point_base[0])
        msg.pose.position.y = float(point_base[1])
        msg.pose.position.z = float(point_base[2])
        msg.pose.orientation.w = 1.0
        self.target_pub.publish(msg)
        self.get_logger().info(f"发布目标: ({point_base[0]:.3f}, {point_base[1]:.3f}, {point_base[2]:.3f})")
```

### Step 2：编译 + 测试

```bash
cd ~/lerobot_ws
colcon build --symlink-install --packages-select position_topic

# 终端1：Gazebo + 相机
ros2 launch lerobot_description so101_gazebo.launch.py

# 终端2：vlm_bridge
ros2 run position_topic vlm_bridge
```

在终端3验证：
```bash
ros2 topic echo /target_pose
# 预期看到每秒发布一条带坐标的消息
```

### Step 3：验证坐标合理性

在 Gazebo 里放一个目标物体在已知位置（比如 target_box 在 `(0.2, 0.0, 0.015)`），手动在 vlm_bridge 里设置像素坐标为图像中那个物体的位置，看输出的 `/target_pose` 是否接近 `(0.2, 0.0, 0.015)`。

> 如果偏差很大（>5cm），检查深度图编码、相机内参、TF 方向。最常见的坑：深度图是毫米还是米、光学坐标系方向（x向右/y向下/z向前）。

### 完成标准

- [ ] `ros2 topic echo /target_pose` 能看到坐标输出
- [ ] 坐标误差 < 5cm（相对于 Gazebo 中物体的实际位置）

---

## Day 4（5月4日）：全链路抓取闭环

**目标**：启动完整系统，从 vlm_bridge 到 MoveIt 到 LinkAttacher 抓取，整个链路跑通。

**预计耗时**：2 小时

### Step 1：调整 move_demo.launch.py

编辑 `position_topic/launch/move_demo.launch.py`，添加 vlm_bridge 节点：

```python
vlm_bridge = Node(
    package='position_topic',
    executable='vlm_bridge',
    name='vlm_bridge',
    output='screen'
)

return LaunchDescription([
    gazebo_launch,
    controller_launch,
    moveit_launch,
    position_subscriber,
    vlm_bridge           # 新增
])
```

### Step 2：启动完整系统

```bash
ros2 launch position_topic move_demo.launch.py
```

这会启动：Gazebo + 控制器 + MoveIt + RViz + position_subscriber + vlm_bridge

### Step 3：验证全链路

1. 在 Gazebo 中确认 target_box 存在，机械臂在 home 位姿
2. vlm_bridge 自动发布 `/target_pose`
3. position_subscriber 收到后调用 MoveIt 规划执行
4. 机械臂运动到目标位置

### Step 4：添加 LinkAttacher 抓取

`position_subscriber` 目前只做了"移动到目标"，没有执行抓取。在 Day 4 先手动测试附着：

```bash
# 机械臂到达目标后，手动调用附着服务
ros2 service call /ATTACHLINK linkattacher_msgs/srv/AttachLink \
  "{model1_name: 'so101', link1_name: 'gripper', model2_name: 'target_box', link2_name: 'box_link'}"

# 然后发布一个抬起目标，看物体是否跟随
ros2 topic pub /target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'base'}, pose: {position: {x: 0.2, y: 0.0, z: 0.15}}}"
```

如果附着成功，物体会随机械臂末端运动。

> **完善抓取流程（后续）**：后续可以修改 position_subscriber，在到达目标后自动调用 LinkAttacher 附着，然后抬起，完成完整的 pick 动作。Day 5 如果有余力可以做。

### 完成标准

- [ ] 机械臂能根据 vlm_bridge 发布的目标运动
- [ ] MoveIt 规划成功（无碰撞报错）
- [ ] LinkAttacher 附着成功，物体随末端运动

---

## Day 5（5月5日）：缓冲 + 收尾

**目标**：补前面卡住的步骤，或往前推进额外功能。

**预计耗时**：1–2 小时

### 如果前面都顺利

选项 A：录一段 GIF
```bash
# 用 GNOME 自带的录屏工具或 peek
peek  # 选择 Gazebo 窗口，录10秒
```

选项 B：完善抓取逻辑
- 在 `position_subscriber` 中，规划成功回调后自动调用 LinkAttacher 附着
- 抬起物体到预设高度
- 发布 place 目标

选项 C：搭一个遮挡场景
- 在 `target_box.sdf` 旁边加几个绿色圆柱体（模拟叶子）
- 测试 VLM（目前还是硬编码）对遮挡目标的响应

### 如果某天卡住了

用这一天补。优先保证 Day 1 和 Day 3 完成——这两个是核心链路。

---

## 技术备忘

### 话题清单

| 话题 | 类型 | 用途 |
|------|------|------|
| `/so101/camera/color/image_raw` | `Image` | eye-in-hand RGB 图像 |
| `/so101/camera/depth/image_raw` | `Image` | eye-in-hand 深度图 |
| `/so101/camera/depth/camera_info` | `CameraInfo` | 相机内参 |
| `/camera/depth/image_raw` | `Image` | 外部 Gemini 深度图 |
| `/target_pose` | `PoseStamped` | 目标位姿（vlm_bridge→position_subscriber） |
| `/ATTACHLINK` | `AttachLink` | 物体附着服务 |
| `/DETACHLINK` | `DetachLink` | 物体释放服务 |

### TF 树

```
world
  └── base
        └── shoulder
              └── upper_arm
                    └── lower_arm
                          └── wrist
                                └── gripper
                                      ├── jaw
                                      └── ee_camera_link
                                            └── ee_camera_optical_link
```

外部相机（独立 TF）：
```
world → camera_link → camera_depth_optical_frame
```

### 反投影公式

```
X_cam = (u - cx) * Z / fx
Y_cam = (v - cy) * Z / fy
Z_cam = Z  (深度值)
```

### 常见坑

1. **深度图编码**：Gazebo `libgazebo_ros_camera.so` 输出 depth 为 `32FC1`（float32，单位米），不是 `16UC1`（uint16，单位毫米）。如果拿到的深度值都是 0 或异常大，先检查 encoding。
2. **TF 查找失败**：`lookup_transform` 的两个 frame 必须同时在 TF 树中。确保 `robot_state_publisher` 已启动。
3. **图像 topic 无数据**：Gazebo 相机插件需要模型被正确 spawn 后才开始发布。检查 Gazebo 窗口里相机模型是否可见。
4. **MoveIt 规划失败**：目标点在机械臂工作空间之外。先手动发布一个明显可达的位姿测试。

---

## 五天之后

如果这个计划全部完成，你手上就有了一个**全链路闭环的抓取 demo**。下一步（5月第2周）是接真实的 VLM API，把硬编码像素坐标替换为 VLM 从图像中识别出的目标位置。
