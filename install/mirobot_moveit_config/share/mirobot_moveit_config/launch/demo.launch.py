from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("mirobot_urdf_2", package_name="mirobot_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)



# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_demo_launch
# from launch_ros.actions import Node

# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder(
#         "mirobot_urdf_2", package_name="mirobot_moveit_config"
#     ).to_moveit_configs()

#     # Add the use_sim_time parameter
#     moveit_parameters = [
#         moveit_config.robot_description,
#         moveit_config.robot_description_semantic,
#         moveit_config.robot_description_kinematics,
#         {"use_sim_time": True},
#     ]

#     # Define your custom node instead of the tutorial node
#     move_program_node = Node(
#         package="move_program",  # Replace with your package name
#         executable="move_program",   # Replace with your executable name
#         output="screen",
#         parameters=moveit_parameters,
#     )

#     # Combine everything into the demo launch
#     return generate_demo_launch(moveit_config) + [move_program_node]



# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_demo_launch
# from launch_ros.actions import Node
# from launch import LaunchDescription

# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder(
#         "mirobot_urdf_2", package_name="mirobot_moveit_config"
#     ).to_moveit_configs()

#     # Add the use_sim_time parameter
#     moveit_parameters = [
#         moveit_config.robot_description,
#         moveit_config.robot_description_semantic,
#         moveit_config.robot_description_kinematics,
#         {"use_sim_time": True},
#     ]

#     # Define your custom node
#     move_program_node = Node(
#         package="move_program",  # Replace with your package name
#         executable="move_program",   # Replace with your executable name
#         output="screen",
#         parameters=moveit_parameters,
#     )

#     # Create and return a combined LaunchDescription
#     return LaunchDescription(
#         generate_demo_launch(moveit_config).entities + [move_program_node]
#     )
