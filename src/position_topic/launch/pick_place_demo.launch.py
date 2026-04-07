import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    object_name_arg = DeclareLaunchArgument("object_name", default_value="target_box")
    object_x_arg = DeclareLaunchArgument("object_x", default_value="-0.16")
    object_y_arg = DeclareLaunchArgument("object_y", default_value="0.02")
    object_z_arg = DeclareLaunchArgument("object_z", default_value="0.14")

    object_name = LaunchConfiguration("object_name")
    object_x = LaunchConfiguration("object_x")
    object_y = LaunchConfiguration("object_y")
    object_z = LaunchConfiguration("object_z")

    lerobot_description_dir = get_package_share_directory("lerobot_description")
    lerobot_controller_dir = get_package_share_directory("lerobot_controller")
    lerobot_moveit_dir = get_package_share_directory("lerobot_moveit")
    position_topic_dir = get_package_share_directory("position_topic")
    link_attacher_lib_dir = os.path.join(get_package_prefix("ros2_linkattacher"), "lib")

    set_gazebo_plugin_path = SetEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=[
            TextSubstitution(text=link_attacher_lib_dir),
            TextSubstitution(text=":"),
            EnvironmentVariable("GAZEBO_PLUGIN_PATH", default_value=""),
        ],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lerobot_description_dir, "launch", "so101_gazebo.launch.py")
        ),
        launch_arguments={
            "world": os.path.join(position_topic_dir, "worlds", "pick_place_attacher.world"),
        }.items(),
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lerobot_controller_dir, "launch", "so101_controller.launch.py")
        ),
        launch_arguments={"is_sim": "True"}.items(),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lerobot_moveit_dir, "launch", "so101_moveit.launch.py")
        ),
        launch_arguments={"is_sim": "True"}.items(),
    )

    pick_place_demo = Node(
        package="position_topic",
        executable="pick_place_demo",
        output="screen",
        parameters=[
            os.path.join(position_topic_dir, "config", "pick_place_demo.yaml"),
            {"object_name": object_name},
            {"object_spawn_x": object_x},
            {"object_spawn_y": object_y},
            {"object_spawn_z": object_z},
            {"object_model_sdf_path": os.path.join(position_topic_dir, "models", "target_box.sdf")},
        ],
    )
    delayed_demo = TimerAction(period=6.0, actions=[pick_place_demo])

    return LaunchDescription([
        object_name_arg,
        object_x_arg,
        object_y_arg,
        object_z_arg,
        set_gazebo_plugin_path,
        gazebo,
        controller,
        moveit,
        delayed_demo,
    ])
