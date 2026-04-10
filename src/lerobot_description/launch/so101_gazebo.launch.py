import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    position_topic = get_package_share_directory("position_topic")
    lerobot_description = get_package_share_directory("lerobot_description")

    world_file = os.path.join(position_topic, "worlds", "pick_place_attacher.world")
    target_file = os.path.join(position_topic, "models", "target_box.sdf")

    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(lerobot_description, "urdf", "so101.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    # 1. 解析 URDF/Xacro
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )

    # 2. 启动 robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}]
    )

    # 3. 启动 Gazebo Classic 服务端和客户端
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
        )]),
        launch_arguments={'world': world_file, 'pause': 'false'}.items()
    )

    # 4. 在 Gazebo Classic 中生成机器人实体
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "so101"
        ],
        output="screen"
    )

    spawn_target = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "target_box",
            "-file", target_file,
            '-x', '0.2', '-y', '0.0', '-z', '0.015'
        ],
        output="screen"
    )

    # 1. 物理位置 (例如布置在正前方 0.3m，高度 0.8m，往下看)
    # 注意：Z=0.8m 减去桌子高度，必须大于 0.25m 的物理盲区！
    spawn_camera = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "gemini_335",
            "-file", os.path.join(position_topic, "models", "camera.sdf"),
            "-x", "0.3", "-y", "0.0", "-z", "0.8", 
            "-R", "0.0", "-P", "1.5708", "-Y", "0.0" 
        ],
        output="screen"
    )

    # 2. Base Link 到 World 的连接 (你在世界里把它装在哪)
    camera_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0.3", "--y", "0.0", "--z", "0.8", 
            "--yaw", "0.0", "--pitch", "1.5708", "--roll", "0.0",
            "--frame-id", "world", 
            "--child-frame-id", "camera_link"
        ]
    )

    # 3. 模拟 Orbbec 的光学坐标系转换 (非常标准的光学坐标系法则)
    camera_optical_tf = Node(
         package="tf2_ros",
         executable="static_transform_publisher",
         arguments=[
             "--x", "0", "--y", "0", "--z", "0",
             "--yaw", "-1.5708", "--pitch", "0", "--roll", "-1.5708",
             "--frame-id", "camera_link", 
             "--child-frame-id", "camera_depth_optical_frame" # 必须与 SDF 中的 frame_name 严格一致
         ]
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        spawn_target,
        spawn_camera,
        camera_base_tf,
        camera_optical_tf
        # 注意：这里彻底删除了 GZ_SIM_RESOURCE_PATH 和 gz_ros2_bridge
    ])