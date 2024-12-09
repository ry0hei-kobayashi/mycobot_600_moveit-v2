from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_spawn_controllers_launch

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from srdfdom.srdf import SRDF

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)


def generate_spawn_controllers_launch(moveit_config):
    controller_names = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])
    print("--------------------------------------------------")
    print("controller_names: ", controller_names)
    print("--------------------------------------------------")
    ld = LaunchDescription()
    for controller in controller_names + ["joint_state_broadcaster"]:
    # for controller in controller_names:
        ld.add_action(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
                output="screen",
            )
        )
    return ld

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("firefighter", package_name="mycobot_600_moveit_config").to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config)
