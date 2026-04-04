from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

# How to run:
# ros2 launch smm_synthesis smm_synthesis_viz_tcp_motion_ndof.launch.py \
#  data_dir:=/home/nikos/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml \
#  xacro_path:=urdf/6dof/smm_structure_anatomy_assembly_6dof.xacro \
#  dof:=6

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
        description="Active DOF of the synthesized SMM structure."
    )
    
    data_dir = LaunchConfiguration("data_dir")
    xacro_path = LaunchConfiguration("xacro_path")
    dof = LaunchConfiguration("dof")

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
                "publish_topic": "/smm_joint_motion_cmd_ndof",
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
                "joint_cmd_topic": "/smm_joint_motion_cmd_ndof",
            }
        ],
    )

    return LaunchDescription([
        data_dir_arg,
        xacro_path_arg,
        dof_arg,
        synthesis_launch,
        joint_gui_node,
        tcp_motion_viz_node,
    ])