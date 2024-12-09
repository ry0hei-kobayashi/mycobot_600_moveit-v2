from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file_path = get_package_share_directory('mycobot_description') + '/urdf/mycobot_pro_600/mycobot_pro_600.urdf'

    return LaunchDescription([
        DeclareLaunchArgument(
            name='urdf_file',
            default_value=urdf_file_path,
            description='Path to the URDF file to load'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('urdf_file')])}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', get_package_share_directory('mycobot_description') + '/rviz/view_mycobot.rviz']
        )
    ])

