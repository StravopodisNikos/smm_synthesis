from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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

    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="base_plate",
        description="Frame id used for ellipsoid messages and RViz markers.",
    )

    run_kinematic_manip_arg = DeclareLaunchArgument(
        "run_kinematic_manipulability",
        default_value="true",
    )

    run_dynamic_manip_arg = DeclareLaunchArgument(
        "run_dynamic_manipulability",
        default_value="true",
    )

    publish_kin_trans_arg = DeclareLaunchArgument(
        "publish_kinematic_translational",
        default_value="true",
    )

    publish_kin_rot_arg = DeclareLaunchArgument(
        "publish_kinematic_rotational",
        default_value="true",
    )

    publish_dyn_trans_arg = DeclareLaunchArgument(
        "publish_dynamic_translational",
        default_value="true",
    )

    publish_dyn_rot_arg = DeclareLaunchArgument(
        "publish_dynamic_rotational",
        default_value="true",
    )

    dynamic_mass_rep_arg = DeclareLaunchArgument(
        "dynamic_mass_matrix_representation",
        default_value="body",
        description="Mass matrix representation for dynamic manipulability: spatial or body",
    )

    dynamic_body_frame_arg = DeclareLaunchArgument(
        "dynamic_body_frame_selection",
        default_value="joint",
        description="Body frame selection for dynamic manipulability when body representation is used: joint or com",
    )

    data_dir = LaunchConfiguration("data_dir")
    xacro_path = LaunchConfiguration("xacro_path")
    dof = LaunchConfiguration("dof")
    joint_cmd_topic = LaunchConfiguration("joint_cmd_topic")
    frame_id = LaunchConfiguration("frame_id")

    run_kinematic_manip = LaunchConfiguration("run_kinematic_manipulability")
    run_dynamic_manip = LaunchConfiguration("run_dynamic_manipulability")

    publish_kin_trans = LaunchConfiguration("publish_kinematic_translational")
    publish_kin_rot = LaunchConfiguration("publish_kinematic_rotational")
    publish_dyn_trans = LaunchConfiguration("publish_dynamic_translational")
    publish_dyn_rot = LaunchConfiguration("publish_dynamic_rotational")

    dynamic_mass_rep = LaunchConfiguration("dynamic_mass_matrix_representation")
    dynamic_body_frame = LaunchConfiguration("dynamic_body_frame_selection")

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
        condition=IfCondition(run_kinematic_manip),
        parameters=[
            {
                "yaml_base_dir": data_dir,
                "joint_cmd_topic": joint_cmd_topic,
                "frame_id": frame_id,
                "output_topic_trans": "/smm/kinematic_manipulability_ellipsoid_trans_ndof",
                "output_topic_rot": "/smm/kinematic_manipulability_ellipsoid_rot_ndof",
                "publish_translational": publish_kin_trans,
                "publish_rotational": publish_kin_rot,
            }
        ],
    )

    dyn_manip_node = Node(
        package="smm_metrics",
        executable="smm_dynamic_manipulability_ndof_node",
        name="smm_dynamic_manipulability_ndof_node",
        output="screen",
        condition=IfCondition(run_dynamic_manip),
        parameters=[
            {
                "yaml_base_dir": data_dir,
                "joint_cmd_topic": joint_cmd_topic,
                "frame_id": frame_id,
                "output_topic_trans": "/smm/dynamic_manipulability_ellipsoid_trans_ndof",
                "output_topic_rot": "/smm/dynamic_manipulability_ellipsoid_rot_ndof",
                "publish_translational": publish_dyn_trans,
                "publish_rotational": publish_dyn_rot,
                "mass_matrix_representation": dynamic_mass_rep,
                "body_frame_selection": dynamic_body_frame,
            }
        ],
    )

    kin_trans_viz_node = Node(
        package="smm_viz_tools",
        executable="smm_ellipsoid_ndof_viz_node",
        name="smm_kinematic_trans_ellipsoid_ndof_viz_node",
        output="screen",
        condition=IfCondition(publish_kin_trans),
        parameters=[
            {
                "input_topic": "/smm/kinematic_manipulability_ellipsoid_trans_ndof",
                "ellipsoid_topic": "/visualization_kin_ell_trans_ndof",
                "axes_topic": "/visualization_kin_ell_trans_axes_ndof",
                "marker_namespace": "kinematic_trans_ellipsoid_ndof",
                "axes_namespace": "kinematic_trans_axes_ndof",
                "ellipsoid_alpha": 0.25,
                "ellipsoid_color_r": 0.871,
                "ellipsoid_color_g": 0.520,
                "ellipsoid_color_b": 0.150,
            }
        ],
    )

    kin_rot_viz_node = Node(
        package="smm_viz_tools",
        executable="smm_ellipsoid_ndof_viz_node",
        name="smm_kinematic_rot_ellipsoid_ndof_viz_node",
        output="screen",
        condition=IfCondition(publish_kin_rot),
        parameters=[
            {
                "input_topic": "/smm/kinematic_manipulability_ellipsoid_rot_ndof",
                "ellipsoid_topic": "/visualization_kin_ell_rot_ndof",
                "axes_topic": "/visualization_kin_ell_rot_axes_ndof",
                "marker_namespace": "kinematic_rot_ellipsoid_ndof",
                "axes_namespace": "kinematic_rot_axes_ndof",
                "ellipsoid_alpha": 0.25,
                "ellipsoid_color_r": 0.80,
                "ellipsoid_color_g": 0.20,
                "ellipsoid_color_b": 0.20,
            }
        ],
    )

    dyn_trans_viz_node = Node(
        package="smm_viz_tools",
        executable="smm_ellipsoid_ndof_viz_node",
        name="smm_dynamic_trans_ellipsoid_ndof_viz_node",
        output="screen",
        condition=IfCondition(publish_dyn_trans),
        parameters=[
            {
                "input_topic": "/smm/dynamic_manipulability_ellipsoid_trans_ndof",
                "ellipsoid_topic": "/visualization_dyn_ell_trans_ndof",
                "axes_topic": "/visualization_dyn_ell_trans_axes_ndof",
                "marker_namespace": "dynamic_trans_ellipsoid_ndof",
                "axes_namespace": "dynamic_trans_axes_ndof",
                "ellipsoid_alpha": 0.25,
                "ellipsoid_color_r": 0.20,
                "ellipsoid_color_g": 0.60,
                "ellipsoid_color_b": 0.90,
            }
        ],
    )

    dyn_rot_viz_node = Node(
        package="smm_viz_tools",
        executable="smm_ellipsoid_ndof_viz_node",
        name="smm_dynamic_rot_ellipsoid_ndof_viz_node",
        output="screen",
        condition=IfCondition(publish_dyn_rot),
        parameters=[
            {
                "input_topic": "/smm/dynamic_manipulability_ellipsoid_rot_ndof",
                "ellipsoid_topic": "/visualization_dyn_ell_rot_ndof",
                "axes_topic": "/visualization_dyn_ell_rot_axes_ndof",
                "marker_namespace": "dynamic_rot_ellipsoid_ndof",
                "axes_namespace": "dynamic_rot_axes_ndof",
                "ellipsoid_alpha": 0.25,
                "ellipsoid_color_r": 0.20,
                "ellipsoid_color_g": 0.30,
                "ellipsoid_color_b": 0.95,
            }
        ],
    )

    return LaunchDescription([
        data_dir_arg,
        xacro_path_arg,
        dof_arg,
        joint_cmd_topic_arg,
        frame_id_arg,
        run_kinematic_manip_arg,
        run_dynamic_manip_arg,
        publish_kin_trans_arg,
        publish_kin_rot_arg,
        publish_dyn_trans_arg,
        publish_dyn_rot_arg,
        dynamic_mass_rep_arg,
        dynamic_body_frame_arg,
        synthesis_launch,
        joint_gui_node,
        tcp_motion_viz_node,
        kin_manip_node,
        dyn_manip_node,
        kin_trans_viz_node,
        kin_rot_viz_node,
        dyn_trans_viz_node,
        dyn_rot_viz_node,
    ])