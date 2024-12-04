from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    # Build the MoveIt configuration
    moveit_config = MoveItConfigsBuilder(
        "mirobot_urdf_2", package_name="mirobot_moveit_config"
    ).to_moveit_configs()

    # Explicitly specify the RViz configuration file path
    rviz_config_path = moveit_config.package_path + "/config/moveit.rviz"

    # Add Move Group Node with kinematics and other parameters
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    # Add RViz node for visualization
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[{"use_sim_time": True}],
    )

    # Return the complete launch description
    return LaunchDescription([move_group_node, rviz_node])
