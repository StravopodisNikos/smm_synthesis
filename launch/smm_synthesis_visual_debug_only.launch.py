from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

"""
===============================================================================
File: smm_synthesis_visual_debug_only.launch.py
Package: smm_synthesis

Purpose
-------
Development / debug launch file for one-shot inspection of SMM N-DOF
kinematics and dynamics class members.

This launch file is intended for controlled debugging of the screw-theory
library internals without the continuous marker publishing and repeated
callbacks of the normal visualization pipeline.

What this launch file starts
----------------------------
1. master_synthesis_ndof.launch.py
   Provides:
     - live YAML generation
     - robot_state_publisher
     - RViz
     - all structure-dependent extracted YAML files

2. smm_joint_motion_gui_ndof_node.py
   Provides:
     - custom GUI sliders for q, dq, ddq
     - publishes:
         /smm_joint_motion_cmd_ndof

3. smm_visual_debug_only_node
   Provides:
     - one-shot or continuous debug execution of selected
       ScrewsKinematicsNdof / ScrewsDynamicsNdof member functions
     - prints matrices / vectors / poses to terminal
     - shuts down after the first valid message by default

How the full pipeline works
---------------------------
master_synthesis_ndof.launch.py
    -> creates the synthesized robot model and YAML files

smm_joint_motion_gui_ndof_node.py
    -> publishes q, dq, ddq-like transport on:
         /smm_joint_motion_cmd_ndof

smm_visual_debug_only_node
    -> waits for a valid GUI command
    -> runs selected library computations
    -> prints all requested internals
    -> exits immediately if debug_once=true

Why this launch file exists
---------------------------
This launch file is useful when debugging:
  - forward kinematics
  - body Jacobians
  - COM Jacobians
  - hybrid / operational Jacobian
  - TCP velocity / acceleration
  - mass matrix

without flooding the console with repeated callbacks.

Default behavior
----------------
By default:
  debug_once := true

So the debug node:
  - processes one valid JointState message
  - prints once
  - shuts down

Later, if repeated execution is needed:
  debug_once := false

Typical usage
-------------
6DOF for debug_once, run_body_jacobians_1:

ros2 launch smm_synthesis smm_synthesis_visual_debug_only.launch.py \
  data_dir:=/home/nikos/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml \
  xacro_path:=urdf/6dof/smm_structure_anatomy_assembly_6dof.xacro \
  dof:=6 \
  debug_once:=true \
  run_body_jacobians_1:=true \
  run_body_jacobians_2:=false \
  run_com_jacobians:=false \
  run_hybrid_jacobian:=false \
  run_mass_matrix:=false \ 
  run_coriolis_matrix:=true \
  run_gravity_vector:=true
===============================================================================
"""


def generate_launch_description():
    default_data_dir = os.path.expanduser(
        "~/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml/"
    )

    # -------------------------------------------------------------------------
    # Launch arguments shared with the synthesis pipeline
    # -------------------------------------------------------------------------
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
        description="Active DOF of the synthesized SMM structure."
    )

    # -------------------------------------------------------------------------
    # Joint command topic and debug behavior
    # -------------------------------------------------------------------------
    joint_cmd_topic_arg = DeclareLaunchArgument(
        "joint_cmd_topic",
        default_value="/smm_joint_motion_cmd_ndof",
        description="Custom GUI joint motion topic used by the debug node.",
    )

    debug_once_arg = DeclareLaunchArgument(
        "debug_once",
        default_value="true",
        description="If true, process the first valid joint command once and shut down.",
    )

    # -------------------------------------------------------------------------
    # Fine-grained debug switches for class-member testing
    # -------------------------------------------------------------------------
    print_tcp_pose_arg = DeclareLaunchArgument(
        "print_tcp_pose",
        default_value="true",
        description="Print TCP pose."
    )

    print_tcp_velocity_arg = DeclareLaunchArgument(
        "print_tcp_velocity",
        default_value="true",
        description="Print TCP velocity."
    )

    print_tcp_acceleration_arg = DeclareLaunchArgument(
        "print_tcp_acceleration",
        default_value="true",
        description="Print TCP acceleration."
    )

    run_body_jacobians_1_arg = DeclareLaunchArgument(
        "run_body_jacobians_1",
        default_value="true",
        description="Execute computeBodyJacobiansFrames1()."
    )

    run_body_jacobians_2_arg = DeclareLaunchArgument(
        "run_body_jacobians_2",
        default_value="false",
        description="Execute computeBodyJacobiansFrames2()."
    )

    run_com_jacobians_arg = DeclareLaunchArgument(
        "run_com_jacobians",
        default_value="false",
        description="Execute ForwardKinematicsCOM() and computeBodyCOMJacobiansFrames()."
    )

    run_hybrid_jacobian_arg = DeclareLaunchArgument(
        "run_hybrid_jacobian",
        default_value="true",
        description="Execute computeHybridJacobianTCP()."
    )

    run_mass_matrix_arg = DeclareLaunchArgument(
        "run_mass_matrix",
        default_value="false",
        description="Execute mass matrix calculation."
    )

    run_coriolis_matrix_arg = DeclareLaunchArgument(
        "run_coriolis_matrix",
        default_value="false",
        description="Execute coriolis matrix calculation."
    )

    run_gravity_vector_arg = DeclareLaunchArgument(
        "run_gravity_vector",
        default_value="false",
        description="Execute gravity vector calculation."
    )

    dynamics_representation_arg = DeclareLaunchArgument(
        "dynamics_representation",
        default_value="body",
        description="Mass matrix representation: 'spatial' or 'body'."
    )

    body_frame_selection_arg = DeclareLaunchArgument(
        "body_frame_selection",
        default_value="joint",
        description="Body frame selection for body mass matrix: 'joint' or 'com'."
    )

    # -------------------------------------------------------------------------
    # Launch configurations
    # -------------------------------------------------------------------------
    data_dir = LaunchConfiguration("data_dir")
    xacro_path = LaunchConfiguration("xacro_path")
    dof = LaunchConfiguration("dof")
    joint_cmd_topic = LaunchConfiguration("joint_cmd_topic")
    debug_once = LaunchConfiguration("debug_once")

    print_tcp_pose = LaunchConfiguration("print_tcp_pose")
    print_tcp_velocity = LaunchConfiguration("print_tcp_velocity")
    print_tcp_acceleration = LaunchConfiguration("print_tcp_acceleration")

    run_body_jacobians_1 = LaunchConfiguration("run_body_jacobians_1")
    run_body_jacobians_2 = LaunchConfiguration("run_body_jacobians_2")
    run_com_jacobians = LaunchConfiguration("run_com_jacobians")
    run_hybrid_jacobian = LaunchConfiguration("run_hybrid_jacobian")
    run_mass_matrix = LaunchConfiguration("run_mass_matrix")
    run_coriolis_matrix = LaunchConfiguration("run_coriolis_matrix")
    run_gravity_vector = LaunchConfiguration("run_gravity_vector")

    dynamics_representation = LaunchConfiguration("dynamics_representation")
    body_frame_selection = LaunchConfiguration("body_frame_selection")

    # -------------------------------------------------------------------------
    # Include main synthesis launch
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
    # GUI joint command node
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
    # Debug-only node
    # -------------------------------------------------------------------------
    visual_debug_only_node = Node(
        package="smm_viz_tools",
        executable="smm_visual_debug_only_node",
        name="smm_visual_debug_only_node",
        output="screen",
        parameters=[
            {
                "yaml_base_dir": data_dir,
                "joint_cmd_topic": joint_cmd_topic,
                "debug_once": debug_once,

                "print_tcp_pose": print_tcp_pose,
                "print_tcp_velocity": print_tcp_velocity,
                "print_tcp_acceleration": print_tcp_acceleration,

                "run_body_jacobians_1": run_body_jacobians_1,
                "run_body_jacobians_2": run_body_jacobians_2,
                "run_com_jacobians": run_com_jacobians,
                "run_hybrid_jacobian": run_hybrid_jacobian,
                "run_mass_matrix": run_mass_matrix,
                "run_coriolis_matrix": run_coriolis_matrix,
                "run_gravity_vector": run_gravity_vector,

                "dynamics_representation": dynamics_representation,
                "body_frame_selection": body_frame_selection,
            }
        ],
    )

    return LaunchDescription([
        data_dir_arg,
        xacro_path_arg,
        dof_arg,
        joint_cmd_topic_arg,
        debug_once_arg,
        print_tcp_pose_arg,
        print_tcp_velocity_arg,
        print_tcp_acceleration_arg,
        run_body_jacobians_1_arg,
        run_body_jacobians_2_arg,
        run_com_jacobians_arg,
        run_hybrid_jacobian_arg,
        run_mass_matrix_arg,
        run_coriolis_matrix_arg,
        run_gravity_vector_arg,
        dynamics_representation_arg,
        body_frame_selection_arg,
        synthesis_launch,
        joint_gui_node,
        visual_debug_only_node,
    ])