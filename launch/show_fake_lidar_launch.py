from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    description = LaunchDescription()

    lidar = Node(
        package='tutorial_ros_podstawy',
        executable='fake_lidar'
    )


    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments = f'-d {get_package_share_directory('tutorial_ros_podstawy')}/config/show_fake_lidar.rviz'
    )

    description.add_action(lidar)
    description.add_action(rviz)
    return description