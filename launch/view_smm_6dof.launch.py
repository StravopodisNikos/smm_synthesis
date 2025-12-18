from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import os


def generate_launch_description():
    # 1. Locate the 6DoF xacro
    pkg_share = FindPackageShare("smm_synthesis").find("smm_synthesis")
    xacro_file = os.path.join(
        pkg_share,
        "urdf",
        "smm_structure_anatomy_assembly_6dof.xacro",
    )

    # 2. Turn xacro into robot_description string
    robot_description = ParameterValue(
        Command(["xacro", " ", xacro_file]),
        value_type=str,
    )

    # 3. Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # 4. Joint State Publisher GUI (sliders)
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    # 5. RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        # if you want a default config:
        # arguments=["-d", os.path.join(pkg_share, "config", "smm_synthesis_config.rviz")],
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            joint_state_publisher_node,
            rviz_node,
        ]
    )
