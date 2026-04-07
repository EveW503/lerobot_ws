import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # 1. 获取包路径
    controller_package_share_dir = get_package_share_directory("lerobot_controller")
    gazebo_package_share_dir = get_package_share_directory("lerobot_description")
    moveit_package_share_dir = get_package_share_directory("lerobot_moveit")

    # 2. 分别为每一个 launch 文件创建 IncludeLaunchDescription 对象
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_package_share_dir, 'launch', 'so101_gazebo.launch.py')
        )
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_package_share_dir, 'launch', 'so101_controller.launch.py')
        )
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_package_share_dir, 'launch', 'so101_moveit.launch.py')
        )
    )

    # 3. 定义你自己的节点


    position_subscriber = Node(
        package='position_topic',
        executable='position_subscriber',
        name='position_subscriber',
        output='screen'
    )

    # 4. 将所有操作添加到 LaunchDescription 列表中返回
    return LaunchDescription([
        gazebo_launch,
        controller_launch,
        moveit_launch,
        position_subscriber
    ])