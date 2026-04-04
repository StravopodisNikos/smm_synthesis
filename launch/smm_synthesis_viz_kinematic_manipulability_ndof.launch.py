"""
===============================================================================
File: smm_synthesis_viz_kinematic_manipulability_ndof.launch.py
Package: smm_synthesis

Purpose
-------
Launch file for the full N-DOF kinematic manipulability ellipsoid pipeline of
the synthesized Serial Metamorphic Manipulator (SMM).

This launch file is intended for interactive testing and RViz visualization of
the TCP kinematic manipulability ellipsoid of the currently synthesized robot
structure.

What this launch file starts
----------------------------
1. The existing synthesis / robot visualization / GUI motion pipeline:
     master_synthesis_ndof.launch.py
   This provides:
     - live YAML generation for the selected synthesized structure
     - robot_state_publisher
     - classic joint_state_publisher_gui (from master launch)
     - RViz
     - all required extracted YAML robot data

2. The custom N-DOF joint motion GUI node from smm_viz_tools:
     smm_joint_motion_gui_ndof_node.py
   This provides:
     - interactive sliders for q, dq, ddq
     - publishing to:
         /smm_joint_motion_cmd_ndof
       and mirroring to:
         /joint_states
   so that both custom metric nodes and RViz robot motion can be driven by the
   same GUI.

3. The N-DOF kinematic manipulability computation node from smm_metrics:
     smm_kinematic_manipulability_ndof_node
   This node:
     - subscribes to the GUI joint motion topic
     - reconstructs the N-DOF robot from live YAML
     - computes the TCP operational Jacobian
     - extracts the translational kinematic manipulability ellipsoid
     - publishes:
         /smm/kinematic_manipulability_ellipsoid_ndof

4. The N-DOF kinematic ellipsoid visualization node from smm_viz_tools:
     smm_kinematic_ellipsoid_ndof_viz_node
   This node:
     - subscribes to:
         /smm/kinematic_manipulability_ellipsoid_ndof
     - converts the ellipsoid parameters to RViz markers
     - publishes:
         /visualization_kin_ell_ndof
         /visualization_kin_ell_axes_ndof

How the full pipeline works
---------------------------
GUI joint motion publisher
    -> publishes q, dq, ddq-like transport

Kinematic manipulability computation node
    -> computes ellipsoid center, axes, orientation

Kinematic ellipsoid visualization node
    -> converts ellipsoid parameters to RViz markers

RViz
    -> displays the robot + ellipsoid + principal axes

Important note about RViz
-------------------------
The manipulability computation node publishes a custom message
(smm_metrics/msg/ManipulabilityEllipsoid), which RViz cannot display directly.

Therefore the visualization node is required as a bridge:
  custom metrics message -> Marker / MarkerArray

Recommended RViz displays
-------------------------
Add the following RViz displays if they are not already saved in your RViz config:
  - RobotModel
  - Marker      topic: /visualization_kin_ell_ndof
  - MarkerArray topic: /visualization_kin_ell_axes_ndof

Launch arguments
----------------
- data_dir:
    Live synthesis YAML folder shared across the synthesis and metric pipeline.

- xacro_path:
    Absolute or package-relative path to the SMM structure xacro.
    Example:
      urdf/3dof/smm_structure_anatomy_assembly_3dof.xacro
      urdf/6dof/smm_structure_anatomy_assembly_6dof.xacro

- dof:
    Active DOF of the chosen synthesized structure.
    This is passed to the custom GUI node.

- joint_cmd_topic:
    Custom GUI motion topic used by the metrics node.

- output_topic:
    Output topic for the computed kinematic manipulability ellipsoid.

- frame_id:
    Reference frame in which the ellipsoid is published and visualized.

Typical usage
-------------
3DOF:
  ros2 launch smm_synthesis smm_synthesis_viz_kinematic_manipulability_ndof.launch.py \
    data_dir:=/home/nikos/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml \
    xacro_path:=urdf/3dof/smm_structure_anatomy_assembly_3dof.xacro \
    dof:=3

6DOF:
  ros2 launch smm_synthesis smm_synthesis_viz_kinematic_manipulability_ndof.launch.py \
    data_dir:=/home/nikos/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml \
    xacro_path:=urdf/6dof/smm_structure_anatomy_assembly_6dof.xacro \
    dof:=6

Notes
-----
- This launch file does not modify master_synthesis_ndof.launch.py.
- It layers the manipulability computation and visualization nodes on top of
  the existing synthesis/testing pipeline.
- If switching between 3DOF and 6DOF, it is recommended to stop previous ROS 2
  processes before relaunching, so stale state does not remain active.
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
    # -------------------------------------------------------------------------
    # 1) Shared launch arguments
    # -------------------------------------------------------------------------
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
        description="Custom GUI joint motion topic used by metrics nodes.",
    )

    output_topic_arg = DeclareLaunchArgument(
        "output_topic",
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
    output_topic = LaunchConfiguration("output_topic")
    frame_id = LaunchConfiguration("frame_id")

    # -------------------------------------------------------------------------
    # 2) Include the existing synthesis/testing master launch
    # -------------------------------------------------------------------------
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

    # -------------------------------------------------------------------------
    # 3) Custom GUI node for q / dq / ddq-like input
    # -------------------------------------------------------------------------
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

    # -------------------------------------------------------------------------
    # 4) N-DOF kinematic manipulability computation node
    # -------------------------------------------------------------------------
    kin_manip_node = Node(
        package="smm_metrics",
        executable="smm_kinematic_manipulability_ndof_node",
        name="smm_kinematic_manipulability_ndof_node",
        output="screen",
        parameters=[
            {
                "yaml_base_dir": data_dir,
                "joint_cmd_topic": joint_cmd_topic,
                "output_topic": output_topic,
                "frame_id": frame_id,
            }
        ],
    )

    # -------------------------------------------------------------------------
    # 5) N-DOF RViz ellipsoid visualization bridge node
    # -------------------------------------------------------------------------
    kin_ell_viz_node = Node(
        package="smm_viz_tools",
        executable="smm_kinematic_ellipsoid_ndof_viz_node",
        name="smm_kinematic_ellipsoid_ndof_viz_node",
        output="screen",
        parameters=[
            {
                "input_topic": output_topic,
                "ellipsoid_topic": "/visualization_kin_ell_ndof",
                "axes_topic": "/visualization_kin_ell_axes_ndof",
                "ellipsoid_alpha": 0.25,
                "axis_shaft_diameter": 0.01,
                "axis_head_diameter": 0.02,
                "axis_head_length": 0.03,
            }
        ],
    )

    # -------------------------------------------------------------------------
    # 6) Launch description
    # -------------------------------------------------------------------------
    return LaunchDescription([
        data_dir_arg,
        xacro_path_arg,
        dof_arg,
        joint_cmd_topic_arg,
        output_topic_arg,
        frame_id_arg,
        synthesis_launch,
        joint_gui_node,
        kin_manip_node,
        kin_ell_viz_node,
    ])