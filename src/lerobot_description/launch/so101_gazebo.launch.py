import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    lerobot_description = get_package_share_directory("lerobot_description")

    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(lerobot_description, "urdf", "so101.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    world_arg = DeclareLaunchArgument(
        name="world",
        default_value="",
        description="Absolute path to Gazebo world file"
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
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
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

    return LaunchDescription([
        model_arg,
        world_arg,
        robot_state_publisher_node,
        gazebo,
        spawn_entity
        # 注意：这里彻底删除了 GZ_SIM_RESOURCE_PATH 和 gz_ros2_bridge
    ])