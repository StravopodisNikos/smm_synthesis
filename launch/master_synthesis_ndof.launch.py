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
    # 1. Base directory - where all synthesis yaml files are stored
    default_data_dir = os.path.expanduser(
        "~/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/ndof/yaml/"
    )
    # 1.1. set arg to overwrite default dir in cli
    data_dir_arg = DeclareLaunchArgument(
        "data_dir",
        default_value=default_data_dir,
        description="Base directory where synthesis YAML files are written.",
    )

    data_dir = LaunchConfiguration("data_dir")

    # 2. Set output file names (args, to overwite from cli)
    frame_output_file_arg = DeclareLaunchArgument(
        "frame_output_file",
        default_value="gsai0.yaml",
        description="filename for active joint frames (gsai0)"
    )
    com_output_file_arg = DeclareLaunchArgument(
        "com_output_file",
        default_value="gsli0.yaml",
        description="filename for active CoM frames (gsli0)"
    )
    inertia_output_file_arg = DeclareLaunchArgument(
        "inertia_output_file",
        default_value="Mscomi0.yaml",
        description="filename for spatial inertia tensor (Mscomi0)"
    )
    tcp_output_file_arg = DeclareLaunchArgument(
        "tcp_output_file",
        default_value="gst0.yaml",
        description="filename for spatial tcp frame (gst0)"
    )
    act_twist_output_file_arg = DeclareLaunchArgument(
        "act_twist_output_file",
        default_value="xi_ai_anat.yaml",
        description="filename for spatial active teists (xi_ai_s_anat)"
    )
    pas_frame_output_file_arg = DeclareLaunchArgument(
        "pas_frame_output_file",
        default_value="gspj0.yaml",
        description="filename for passive joint frames (gspj0)"
    )
    pas_twist_output_file_arg = DeclareLaunchArgument(
        "pas_twist_output_file",
        default_value="xi_pj_anat.yaml",
        description="filename for spatial active teists (xi_pj_s_anat)"
    )
    pseudo_angle_output_file_arg = DeclareLaunchArgument(
        "pseudo_angle_output_file",
        default_value="q_pj_anat.yaml",
        description="filename for pseudo joint angles (q_pj_anat)"
    )

    # 3. Turn the args in LaunchConfiguration objects
    frame_output_file = LaunchConfiguration("frame_output_file")
    com_output_file = LaunchConfiguration("com_output_file")
    inertia_output_file = LaunchConfiguration("inertia_output_file")
    tcp_output_file = LaunchConfiguration("tcp_output_file")
    act_twist_output_file = LaunchConfiguration("act_twist_output_file")
    pas_frame_output_file = LaunchConfiguration("pas_frame_output_file")
    pas_twist_output_file = LaunchConfiguration("pas_twist_output_file")
    pseudo_angle_output_file = LaunchConfiguration("pseudo_angle_output_file")

    # 4. Locate main robot xacro file: THIS CHANGES BASED ON ROBOT CONSTRUCTED
    # - 4.1 AVAILABLE FOR SMM SUBCLASS (3DOF): smm_structure_anatomy_assembly.xacro
    # - 4.2 NEXT CREATE SIMPLE TEST WITH CUBIC CONNECTORS
    pkg_share = FindPackageShare("smm_synthesis").find("smm_synthesis")
    xacro_file = os.path.join(
        pkg_share,
        "urdf",
        "smm_structure_anatomy_assembly_6dof.xacro", 
    )

    # 5. Assign the robot_description from xacro
    robot_description = ParameterValue(
        Command(["xacro", " ", xacro_file]),
        value_type=str,
    )

    # 6. Build full paths for the extractors’ outputs
    frame_yaml_path = PathJoinSubstitution([data_dir, frame_output_file])
    com_yaml_path   = PathJoinSubstitution([data_dir, com_output_file])
    inertia_yaml_path   = PathJoinSubstitution([data_dir, inertia_output_file])
    tcp_yaml_path   = PathJoinSubstitution([data_dir, tcp_output_file])
    act_twist_yaml_path = PathJoinSubstitution([data_dir, act_twist_output_file])
    pas_frame_yaml_path = PathJoinSubstitution([data_dir, pas_frame_output_file])    
    pas_twist_yaml_path = PathJoinSubstitution([data_dir, pas_twist_output_file])
    pseudo_angle_yaml_path = PathJoinSubstitution([data_dir, pseudo_angle_output_file])

    # 7. Call node list
    # 7.1 robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # 7.2 joint_state_publisher_gui to move joints
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    # 7.3 RViz2 for visualization
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        #arguments=["-d", os.path.join(pkg_share, "config", "smm_synthesis_config.rviz")],
    )

    # 7.4 active_frames_extractor_ndof 
    active_frames_extractor_ndof_node = Node(
        package="smm_synthesis",
        executable="active_frames_extractor_ndof",
        name="active_frames_extractor_ndof",
        output="screen",
        parameters=[
            {"output_file": frame_yaml_path},          # FULL PATH
            {"robot_description": robot_description},  # URDF as string
            {"root_link": "base_link"},
            {"tip_link": "tcp"},
            {"apply_rotz_minus_pi": True},
        ],
    )

    # 7.5 com_frames_extractor_ndof
    com_extractor_ndof_node = Node(
        package="smm_synthesis",
        executable="com_extractor_kdl_ndof",
        name="com_extractor_kdl_ndof",
        output="screen",
        parameters=[
            {"output_file": com_yaml_path},
            {"robot_description": robot_description},
            {"tip_link": "tcp"},
            {"base_link_name": "base_link"},
        ],
    )

    # 7.6 inertia_extractor_ndof
    inertia_extractor_ndof_node = Node(
        package="smm_synthesis",
        executable="inertia_extractor_kdl_ndof",
        name="inertia_extractor_kdl_ndof",
        output="screen",
        parameters=[
            {"output_file": inertia_yaml_path},
            {"robot_description": robot_description},
        ],
    )

    # 7.7 tcp extractor saves the gst0.yaml
    tcp_extractor_ndof_node = Node(
        package="smm_synthesis",
        executable="tcp_extractor_kdl_ndof",
        name="tcp_extractor_kdl_ndof",
        output="screen",
        parameters=[
            {"output_file": tcp_yaml_path},            # FULL PATH
            {"robot_description": robot_description},  # same URDF string
            {"tip_link": "tcp"},
            {"base_link": "base_link"},
        ],
    )

    # 7.8 active twists extractor saves the xi_ai_anat.yaml
    act_twist_extractor_ndof_node = Node(
        package="smm_synthesis",
        executable="twist_extractor_screws_ndof",
        name="twist_extractor_screws_ndof",
        output="screen",
        parameters=[
            {"input_file": frame_yaml_path},            # FULL PATH
            {"output_file": act_twist_yaml_path},            # FULL PATH
        ],
    )

    # 7.9 passive frame_extractor_kdl
    pas_frame_extractor_ndof_node = Node(
        package="smm_synthesis",
        executable="passive_frame_extractor_kdl_ndof",
        name="passive_frame_extractor_kdl_ndof",
        output="screen",
        parameters=[
            {"output_file": pas_frame_yaml_path},          # FULL PATH
            {"robot_description": robot_description},  # URDF as string
        ],
    )

    # 7.10 passive twists extractor saves the xi_pj_anat.yaml
    pas_twist_extractor_ndof_node = Node(
        package="smm_synthesis",
        executable="passive_twist_extractor_screws_ndof",
        name="passive_twist_extractor_screws_ndof",
        output="screen",
        parameters=[
            {"input_file": pas_frame_yaml_path},            # FULL PATH
            {"output_file": pas_twist_yaml_path},            # FULL PATH
        ],
    )

    # 7.11 passive angle extractor saves the q_pj_anat.yaml
    pseudo_angle_extractor_ndof_node = Node(
        package="smm_synthesis",
        executable="pseudo_angle_extractor_ndof",
        name="pseudo_angle_extractor_ndof",
        output="screen",
        parameters=[
            {"output_file": pseudo_angle_yaml_path},
        ],
    )

    # 7.12 structure digit set spnning node to keep param alive in ros2
    structure_digit_ndof_node = Node(
        package="smm_synthesis",
        executable="structure_digit_setter_ndof",
        name="structure_digit_setter_ndof",
        output="screen",
    )

    return LaunchDescription(
        [
            # launch args
            data_dir_arg,
            frame_output_file_arg,
            com_output_file_arg,
            inertia_output_file_arg,
            tcp_output_file_arg,
            act_twist_output_file_arg,
            pas_frame_output_file_arg,
            pas_twist_output_file_arg,
            pseudo_angle_output_file_arg,

            # nodes
            robot_state_publisher_node,
            joint_state_publisher_node,
            rviz_node,
            active_frames_extractor_ndof_node,
            com_extractor_ndof_node,
            inertia_extractor_ndof_node,
            tcp_extractor_ndof_node,
            act_twist_extractor_ndof_node,
            pas_frame_extractor_ndof_node,
            pas_twist_extractor_ndof_node,
            pseudo_angle_extractor_ndof_node,
            structure_digit_ndof_node,

        ]
    )
