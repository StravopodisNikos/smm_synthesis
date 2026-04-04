"""
===============================================================================
File: smm_synthesis_viz_tcp_and_kinematic_manipulability_ndof.launch.py
Package: smm_synthesis

Purpose
-------
Combined N-DOF launch file for interactive visualization of:

  1. TCP velocity and acceleration
  2. TCP kinematic manipulability ellipsoid

for the synthesized Serial Metamorphic Manipulator (SMM).

This launch file extends the existing synthesis/testing pipeline by adding:
  - custom GUI joint motion input
  - TCP motion visualization
  - kinematic manipulability computation
  - kinematic ellipsoid RViz visualization

What this launch file starts
----------------------------
1. master_synthesis_ndof.launch.py
   Provides:
     - live YAML generation
     - robot_state_publisher
     - RViz
     - classic joint_state_publisher_gui from the legacy testing pipeline
     - all structure-dependent extracted YAML files

2. smm_joint_motion_gui_ndof_node.py
   Provides:
     - custom GUI sliders for q, dq, ddq
     - publishes:
         /smm_joint_motion_cmd_ndof
       and mirrors to:
         /joint_states

3. smm_tcp_motion_ndof_viz_node
   Computes and visualizes:
     - TCP linear velocity
     - TCP linear acceleration

4. smm_kinematic_manipulability_ndof_node
   Computes and publishes:
     - kinematic manipulability ellipsoid parameters

5. smm_kinematic_ellipsoid_ndof_viz_node
   Converts ellipsoid parameters to RViz markers

How the full pipeline works
---------------------------
GUI joint motion publisher
    -> publishes q, dq, ddq-like transport

TCP motion visualization node
    -> computes TCP velocity and acceleration
    -> publishes RViz arrows

Kinematic manipulability computation node
    -> computes ellipsoid center / axes / orientation
    -> publishes custom ellipsoid message

Kinematic ellipsoid visualization node
    -> converts custom ellipsoid message to Marker / MarkerArray

Recommended RViz displays
-------------------------
Add these RViz displays if not already present:
  - RobotModel
  - MarkerArray topic: /visualization_tcp_motion_ndof
  - Marker      topic: /visualization_kin_ell_ndof
  - MarkerArray topic: /visualization_kin_ell_axes_ndof

Typical usage
-------------
3DOF:
  ros2 launch smm_synthesis smm_synthesis_viz_tcp_and_kinematic_manipulability_ndof.launch.py \
    data_dir:=/home/nikos/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml \
    xacro_path:=urdf/3dof/smm_structure_anatomy_assembly_3dof.xacro \
    dof:=3

6DOF:
  ros2 launch smm_synthesis smm_synthesis_viz_tcp_and_kinematic_manipulability_ndof.launch.py \
    data_dir:=/home/nikos/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml \
    xacro_path:=urdf/6dof/smm_structure_anatomy_assembly_6dof.xacro \
    dof:=6
===============================================================================
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    default_data_dir = os.path.expanduser(
        "~/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml/"
    )

    data_dir_arg = DeclareLaunchArgument(
        "data_dir",
        default_value=default_data_dir,
        description="Base directory where synthesis YAML files are written.",
    )

    xacro_path_arg = DeclareLaunchArgument(
        "xacro_path",
        default_value="urdf/6dof/smm_structure_anatomy_assembly_6dof.xacro",
        description="Absolute or package-relative path to the SMM structure xacro.",
    )

    dof_arg = DeclareLaunchArgument(
        "dof",
        default_value="6",
        description="Active DOF of the synthesized SMM structure.",
    )

    joint_cmd_topic_arg = DeclareLaunchArgument(
        "joint_cmd_topic",
        default_value="/smm_joint_motion_cmd_ndof",
        description="Custom GUI joint motion topic used by visualization and metrics nodes.",
    )

    ellipsoid_output_topic_arg = DeclareLaunchArgument(
        "ellipsoid_output_topic",
        default_value="/smm/kinematic_manipulability_ellipsoid_ndof",
        description="Output topic for the computed kinematic manipulability ellipsoid.",
    )

    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="base_plate",
        description="Frame id used for the ellipsoid message and RViz markers.",
    )

    data_dir = LaunchConfiguration("data_dir")
    xacro_path = LaunchConfiguration("xacro_path")
    dof = LaunchConfiguration("dof")
    joint_cmd_topic = LaunchConfiguration("joint_cmd_topic")
    ellipsoid_output_topic = LaunchConfiguration("ellipsoid_output_topic")
    frame_id = LaunchConfiguration("frame_id")

    smm_synthesis_share = get_package_share_directory("smm_synthesis")
    master_launch_path = os.path.join(
        smm_synthesis_share, "launch", "master_synthesis_ndof.launch.py"
    )

    synthesis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(master_launch_path),
        launch_arguments={
            "data_dir": data_dir,
            "xacro_path": xacro_path,
        }.items(),
    )

    joint_gui_node = Node(
        package="smm_viz_tools",
        executable="smm_joint_motion_gui_ndof_node.py",
        name="smm_joint_motion_gui_ndof_node",
        output="screen",
        parameters=[
            {
                "dof": dof,
                "publish_topic": joint_cmd_topic,
                "publish_rate_hz": 20.0,
                "q_min": -3.14,
                "q_max": 3.14,
                "dq_min": -2.0,
                "dq_max": 2.0,
                "ddq_min": -5.0,
                "ddq_max": 5.0,
            }
        ],
    )

    tcp_motion_viz_node = Node(
        package="smm_viz_tools",
        executable="smm_tcp_motion_ndof_viz_node",
        name="smm_tcp_motion_ndof_viz_node",
        output="screen",
        parameters=[
            {
                "yaml_base_dir": data_dir,
                "base_frame": "base_plate",
                "velocity_scale": 0.25,
                "acceleration_scale": 0.10,
                "joint_cmd_topic": joint_cmd_topic,
            }
        ],
    )

    kin_manip_node = Node(
        package="smm_metrics",
        executable="smm_kinematic_manipulability_ndof_node",
        name="smm_kinematic_manipulability_ndof_node",
        output="screen",
        parameters=[
            {
                "yaml_base_dir": data_dir,
                "joint_cmd_topic": joint_cmd_topic,
                "output_topic": ellipsoid_output_topic,
                "frame_id": frame_id,
            }
        ],
    )

    kin_ell_viz_node = Node(
        package="smm_viz_tools",
        executable="smm_kinematic_ellipsoid_ndof_viz_node",
        name="smm_kinematic_ellipsoid_ndof_viz_node",
        output="screen",
        parameters=[
            {
                "input_topic": ellipsoid_output_topic,
                "ellipsoid_topic": "/visualization_kin_ell_ndof",
                "axes_topic": "/visualization_kin_ell_axes_ndof",
                "ellipsoid_alpha": 0.25,
                "axis_shaft_diameter": 0.01,
                "axis_head_diameter": 0.02,
                "axis_head_length": 0.03,
            }
        ],
    )

    return LaunchDescription([
        data_dir_arg,
        xacro_path_arg,
        dof_arg,
        joint_cmd_topic_arg,
        ellipsoid_output_topic_arg,
        frame_id_arg,
        synthesis_launch,
        joint_gui_node,
        tcp_motion_viz_node,
        kin_manip_node,
        kin_ell_viz_node,
    ])