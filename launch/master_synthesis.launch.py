from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os


# HOW TO USE:
#   Default data_dir:
#       ~/ros2_ws/src/smm_data/synthesis/yaml
#
#   Override:
#       ros2 launch smm_synthesis master_synthesis_launch.py \
#         data_dir:=/home/nikos/custom_smm_data/synthesis/yaml


def generate_launch_description():
    # 1) Argument: where to store YAMLs (default = your smm_data folder)
    default_data_dir = os.path.expanduser(
        "~/ros2_ws/src/smm_data/synthesis/yaml"
    )

    data_dir_arg = DeclareLaunchArgument(
        "data_dir",
        default_value=default_data_dir,
        description="Base directory where synthesis YAML files are written.",
    )

    data_dir = LaunchConfiguration("data_dir")

    # 2) Locate your xacro
    pkg_share = FindPackageShare("smm_synthesis").find("smm_synthesis")
    xacro_file = os.path.join(
        pkg_share,
        "urdf",
        "smm_structure_anatomy_assembly.xacro",
    )

    # 3) robot_description from xacro
    robot_description = ParameterValue(
        Command(["xacro", " ", xacro_file]),
        value_type=str,
    )

    # 4) Build full paths for the extractors’ outputs
    frame_yaml_path = PathJoinSubstitution([data_dir, "gsai0.yaml"])
    com_yaml_path   = PathJoinSubstitution([data_dir, "gsli0.yaml"])

    # 5) robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # 6) joint_state_publisher_gui to move joints
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    # 7) RViz2 for visualization
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        #arguments=["-d", os.path.join(pkg_share, "config", "smm_synthesis_config.rviz")],
    )

    # 8) frame_extractor_kdl – SAME logic as your working extract_frames.launch.py
    frame_extractor_node = Node(
        package="smm_synthesis",
        executable="frame_extractor_kdl",
        name="frame_extractor_kdl",
        output="screen",
        parameters=[
            {"output_file": frame_yaml_path},          # FULL PATH
            {"robot_description": robot_description},  # URDF as string
        ],
    )

    # 9) NEW: com_extractor_kdl – writes COMs to gsli0.yaml
    com_extractor_node = Node(
        package="smm_synthesis",
        executable="com_extractor_kdl",
        name="com_extractor_kdl",
        output="screen",
        parameters=[
            {"output_file": com_yaml_path},            # FULL PATH
            {"robot_description": robot_description},  # same URDF string
        ],
    )

    return LaunchDescription(
        [
            data_dir_arg,
            robot_state_publisher_node,
            joint_state_publisher_node,
            rviz_node,
            frame_extractor_node,
            com_extractor_node,
        ]
    )
