from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # Where smm_synthesis’s installed stuff lives
    pkg_share = get_package_share_directory("smm_synthesis")

    # --- 1. data_dir argument (same semantics as in master_synthesis.launch.py) ---
    default_data_dir = os.path.expanduser(
        "~/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/3dof/yaml/"
    )

    data_dir_arg = DeclareLaunchArgument(
        "data_dir",
        default_value=default_data_dir,
        description="Base directory where synthesis YAML files are written.",
    )

    data_dir = LaunchConfiguration("data_dir")

    # --- 2. Include your existing master_synthesis.launch.py ---
    master_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "master_synthesis.launch.py")
            # ^ if your file is named differently, change this line
        ),
        # Forward the same data_dir arg into the included launch
        launch_arguments={
            "data_dir": data_dir,
        }.items(),
    )

    # --- 3. Add the twists visualization node from smm_viz_tools ---
    # We reuse *the same* data_dir as yaml_base_dir, so it matches your synthesis outputs.
    twists_viz_node = Node(
        package="smm_viz_tools",
        executable="smm_twists2_viz_node",
        name="smm_twists_viz2_node",
        output="screen",
        parameters=[
            {
                "yaml_base_dir": data_dir,
            }
        ],
    )

    return LaunchDescription(
        [
            data_dir_arg,   # declare the arg at top-level
            master_launch,  # run the synthesis pipeline
            twists_viz_node # run the twists visualizer
        ]
    )
