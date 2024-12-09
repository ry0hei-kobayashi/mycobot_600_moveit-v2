from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_warehouse_db_launch

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

def generate_warehousr_db_launch(moveit_config):
    """Launch file for warehouse database"""
    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "moveit_warehouse_database_path",
            default_value=str(
                moveit_config.package_path / "default_warehouse_mongo_db"
            ),
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("reset", default_value=False))

    # The default DB port for moveit (not default MongoDB port to avoid potential conflicts)
    ld.add_action(DeclareLaunchArgument("moveit_warehouse_port", default_value="33829"))

    # The default DB host for moveit
    ld.add_action(
        DeclareLaunchArgument("moveit_warehouse_host", default_value="localhost")
    )

    # Load warehouse parameters
    db_parameters = [
        {
            "overwrite": False,
            "database_path": LaunchConfiguration("moveit_warehouse_database_path"),
            "warehouse_port": LaunchConfiguration("moveit_warehouse_port"),
            "warehouse_host": LaunchConfiguration("moveit_warehouse_host"),
            "warehouse_exec": "mongod",
            "warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection",
        },
    ]
    # Run the DB server
    db_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        # TODO(dlu): Figure out if this needs to be run in a specific directory
        # (ROS 1 version set cwd="ROS_HOME")
        parameters=db_parameters,
    )
    ld.add_action(db_node)

    # If we want to reset the database, run this node
    reset_node = Node(
        package="moveit_ros_warehouse",
        executable="moveit_init_demo_warehouse",
        output="screen",
        condition=IfCondition(LaunchConfiguration("reset")),
    )
    ld.add_action(reset_node)

    return ld

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("firefighter", package_name="mycobot_600_moveit_config").to_moveit_configs()
    return generate_warehouse_db_launch(moveit_config)
