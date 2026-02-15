from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

# How to use:
# $ ros2 ros2 launch smm_synthesis smm_synthesis_viz_twists_ndof.launch.py \
#  data_dir:=/home/nikos/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml \
#  xacro_path:=urdf/6dof/smm_structure_anatomy_assembly_6dof.xacro
# $ ros2 launch smm_synthesis smm_synthesis_viz_twists_ndof.launch.py   data_dir:=/home/nikos/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml   xacro_path:=urdf/3dof/smm_structure_anatomy_assembly_3dof.xacro

def generate_launch_description():
    # --- 1. Common arguments: data_dir and xacro_path ---
    # data_dir: live synthesis YAML folder (same as master_synthesis_ndof)
    default_data_dir = os.path.expanduser(
        "~/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml/"
    )

    data_dir_arg = DeclareLaunchArgument(
        "data_dir",
        default_value=default_data_dir,
        description="Base directory where synthesis YAML files are written (live folder).",
    )

    # xacro_path: which structure (3dof / 6dof) you want to visualize
    xacro_path_arg = DeclareLaunchArgument(
        "xacro_path",
        default_value="urdf/6dof/smm_structure_anatomy_assembly_6dof.xacro",
        description=(
            "Absolute or package-relative path to the SMM structure xacro. "
            "DOF is inferred from the filename pattern '*_Xdof.xacro'."
        ),
    )

    data_dir = LaunchConfiguration("data_dir")
    xacro_path = LaunchConfiguration("xacro_path")

    # --- 2. Include the existing master_synthesis_ndof.launch.py ---
    smm_synthesis_share = get_package_share_directory("smm_synthesis")
    master_launch_path = os.path.join(
        smm_synthesis_share, "launch", "master_synthesis_ndof.launch.py"
    )

    synthesis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(master_launch_path),
        launch_arguments={
            "data_dir": data_dir,
            "xacro_path": xacro_path,
            # All other output filenames use their defaults inside master_synthesis_ndof
        }.items(),
    )

    # --- 3. Twists visualization node (Jacobian-based) ---
    twists_viz_node = Node(
        package="smm_viz_tools",
        executable="smm_twists_ndof_viz_node",  # from your comment in the cpp header
        name="smm_twists_ndof_viz_node",
        output="screen",
        parameters=[
            {
                # Same live YAML folder as used by synthesis
                "yaml_base_dir": data_dir
            }
        ],
    )

    return LaunchDescription(
        [
            data_dir_arg,
            xacro_path_arg,
            synthesis_launch,
            twists_viz_node,
        ]
    )
