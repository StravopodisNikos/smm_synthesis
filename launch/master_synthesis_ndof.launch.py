from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch.event_handlers import OnProcessExit

from ament_index_python.packages import get_package_share_directory

import os
import re
import shutil


# How to test the 6-dof visualization:
# $ ros2 launch smm_synthesis master_synthesis_ndof.launch.py \
#   data_dir:=/home/nikos/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml \
#   xacro_path:=/home/nikos/ros2_ws/src/smm_class_pkgs/smm_synthesis/urdf/6dof/smm_structure_anatomy_assembly_6dof.xacro
# How to test the 3-dof visualization:
# $ ros2 launch smm_synthesis master_synthesis_ndof.launch.py \
#   data_dir:=/home/nikos/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml \
#   xacro_path:=/home/nikos/ros2_ws/src/smm_class_pkgs/smm_synthesis/urdf/3dof/smm_structure_anatomy_assembly_3dof.xacro


def launch_setup(context, *args, **kwargs):

    # ---------- 1. Resolve package share & arguments ----------
    pkg_share = get_package_share_directory("smm_synthesis")

    data_dir = LaunchConfiguration("data_dir").perform(context)
    xacro_path_arg = LaunchConfiguration("xacro_path").perform(context)

    # Ensure data_dir exists
    if not os.path.isabs(data_dir):
        data_dir = os.path.expanduser(data_dir)
    os.makedirs(data_dir, exist_ok=True)

    # Resolve xacro absolute path
    if os.path.isabs(xacro_path_arg):
        xacro_file = xacro_path_arg
    else:
        # treat as path relative to smm_synthesis share
        xacro_file = os.path.join(pkg_share, xacro_path_arg)

    if not os.path.exists(xacro_file):
        raise RuntimeError(f"[master_synthesis_ndof] xacro file not found: {xacro_file}")

    # ---------- 2. Infer DOF from xacro filename ----------
    # Expect something like "..._3dof.xacro", "..._6dof.xacro"
    basename = os.path.basename(xacro_file)
    m = re.search(r"([3-6])dof", basename)
    if not m:
        raise RuntimeError(
            f"[master_synthesis_ndof] Cannot infer DOF from xacro filename '{basename}'. "
            "Expected pattern like '*_3dof.xacro' or '*_6dof.xacro'."
        )
    dof = int(m.group(1))
    dof_tag = f"{dof}dof"

    # ---------- 3. Copy assembly template → live assembly.yaml ----------
    # Try installed location first:
    #   <install>/share/smm_synthesis/config/yaml/<Xdof>/assembly_<Xdof>.yaml
    assembly_template = os.path.join(
        pkg_share, "config", "yaml", dof_tag, f"assembly_{dof_tag}.yaml"
    )

    if not os.path.exists(assembly_template):
        # Fallback: use the *source* layout relative to the xacro file.
        # xacro_file = .../smm_synthesis/urdf/6dof/smm_structure_anatomy_assembly_6dof.xacro
        # pkg_root   = .../smm_synthesis
        pkg_root = os.path.dirname(os.path.dirname(os.path.dirname(xacro_file)))
        alt_template = os.path.join(
            pkg_root, "config", "yaml", dof_tag, f"assembly_{dof_tag}.yaml"
        )

        if os.path.exists(alt_template):
            assembly_template = alt_template
        else:
            raise RuntimeError(
                "[master_synthesis_ndof] Assembly template not found in either:\n"
                f"  {os.path.join(pkg_share, 'config', 'yaml', dof_tag, f'assembly_{dof_tag}.yaml')}\n"
                f"  {alt_template}"
            )

    live_assembly = os.path.join(data_dir, "assembly.yaml")
    shutil.copyfile(assembly_template, live_assembly)

    print(f"[master_synthesis_ndof] DOF={dof}, using template:")
    print(f"  {assembly_template}  →  {live_assembly}")

    # ---------- 4. Resolve output file names ----------
    frame_output_file   = LaunchConfiguration("frame_output_file").perform(context)
    com_output_file     = LaunchConfiguration("com_output_file").perform(context)
    inertia_output_file = LaunchConfiguration("inertia_output_file").perform(context)
    tcp_output_file     = LaunchConfiguration("tcp_output_file").perform(context)
    act_twist_output    = LaunchConfiguration("act_twist_output_file").perform(context)
    pas_frame_output    = LaunchConfiguration("pas_frame_output_file").perform(context)
    pas_twist_output    = LaunchConfiguration("pas_twist_output_file").perform(context)
    pseudo_angle_output = LaunchConfiguration("pseudo_angle_output_file").perform(context)

    frame_yaml_path        = os.path.join(data_dir, frame_output_file)
    com_yaml_path          = os.path.join(data_dir, com_output_file)
    inertia_yaml_path      = os.path.join(data_dir, inertia_output_file)
    tcp_yaml_path          = os.path.join(data_dir, tcp_output_file)
    act_twist_yaml_path    = os.path.join(data_dir, act_twist_output)
    pas_frame_yaml_path    = os.path.join(data_dir, pas_frame_output)
    pas_twist_yaml_path    = os.path.join(data_dir, pas_twist_output)
    pseudo_angle_yaml_path = os.path.join(data_dir, pseudo_angle_output)

    # ---------- 5. robot_description from xacro ----------
    robot_description = ParameterValue(
        Command(["xacro", " ", xacro_file]),
        value_type=str,
    )

    # ---------- 6. Nodes (Ndof version) ----------
    # 6.1 robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # 6.2 joint_state_publisher_gui to move joints
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    # 6.3 RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        # arguments=["-d", os.path.join(pkg_share, "config", "smm_synthesis_ndof.rviz")],
    )

    # 6.4 active_frames_extractor_ndof → gsai0.yaml
    active_frames_extractor_ndof_node = Node(
        package="smm_synthesis",
        executable="active_frames_extractor_ndof",
        name="active_frames_extractor_ndof",
        output="screen",
        parameters=[
            {"output_file": frame_yaml_path},
            {"robot_description": robot_description},
            {"root_link": "base_plate"}, 
            {"tip_link": "tcp"},
            {"apply_rotz_minus_pi": False}, # se to False for root->base_plate
        ],
    )

    # 6.5 com_extractor_kdl_ndof → gsli0.yaml
    com_extractor_ndof_node = Node(
        package="smm_synthesis",
        executable="com_extractor_kdl_ndof",
        name="com_extractor_kdl_ndof",
        output="screen",
        parameters=[
            {"output_file": com_yaml_path},
            {"robot_description": robot_description},
            {"tip_link": "tcp"},
            {"root_link_name": "base_plate"}, 
        ],
    )

    # 6.6 inertia_extractor_kdl_ndof → Mscomi0.yaml
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

    # 6.7 tcp_extractor_kdl_ndof → gst0.yaml
    tcp_extractor_ndof_node = Node(
        package="smm_synthesis",
        executable="tcp_extractor_kdl_ndof",
        name="tcp_extractor_kdl_ndof",
        output="screen",
        parameters=[
            {"output_file": tcp_yaml_path},
            {"robot_description": robot_description},
            {"tip_link": "tcp"},
            {"root_link": "base_plate"}, 
        ],
    )

    # 6.8 active twists extractor (xi_ai_anat.yaml), needs gsai0.yaml
    act_twist_extractor_ndof_node = Node(
        package="smm_synthesis",
        executable="twist_extractor_screws_ndof",
        name="twist_extractor_screws_ndof",
        output="screen",
        parameters=[
            {"input_file": frame_yaml_path},     # gsai0.yaml
            {"output_file": act_twist_yaml_path},
        ],
    )

    # 6.9 passive frame extractor → gspj0.yaml
    pas_frame_extractor_ndof_node = Node(
        package="smm_synthesis",
        executable="passive_frame_extractor_kdl_ndof",
        name="passive_frame_extractor_kdl_ndof",
        output="screen",
        parameters=[
            {"output_file": pas_frame_yaml_path},   # gspj0.yaml
            {"robot_description": robot_description},
            {"assembly_yaml": assembly_template},   # DOF-specific TEMPLATE
        ],
    )

    # 6.10 passive twists extractor (xi_pj_anat.yaml), needs gspj0.yaml
    pas_twist_extractor_ndof_node = Node(
        package="smm_synthesis",
        executable="passive_twist_extractor_screws_ndof",
        name="passive_twist_extractor_screws_ndof",
        output="screen",
        parameters=[
            {"input_file": pas_frame_yaml_path},    # live gspj0.yaml
            {"output_file": pas_twist_yaml_path},   # live xi_pj_anat.yaml
            {"assembly_yaml": assembly_template},   # DOF-specific TEMPLATE
        ],
    )

    # 6.11 pseudo-angle extractor (q_pj_anat.yaml)
    pseudo_angle_extractor_ndof_node = Node(
        package="smm_synthesis",
        executable="pseudo_angle_extractor_ndof",
        name="pseudo_angle_extractor_ndof",
        output="screen",
        parameters=[
            {"output_file": pseudo_angle_yaml_path},  # live q_pj_anat.yaml
            {"assembly_yaml": assembly_template},     # DOF-specific TEMPLATE
        ],
    )

    # 6.12 structure digit setter (keeps params alive)
    structure_digit_ndof_node = Node(
        package="smm_synthesis",
        executable="structure_digit_setter_ndof",
        name="structure_digit_setter_ndof",
        output="screen",
    )

    # ---------- 7. Chaining: twists AFTER frames ----------
    # Active twists after active frames
    chain_active_twists = RegisterEventHandler(
        OnProcessExit(
            target_action=active_frames_extractor_ndof_node,
            on_exit=[act_twist_extractor_ndof_node],
        )
    )

    # Passive twists after passive frames
    chain_passive_twists = RegisterEventHandler(
        OnProcessExit(
            target_action=pas_frame_extractor_ndof_node,
            on_exit=[pas_twist_extractor_ndof_node],
        )
    )

    # IMPORTANT: we do NOT return act_twist_extractor_ndof_node and
    # pas_twist_extractor_ndof_node directly here, they are launched
    # only through the event handlers above.

    return [
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        active_frames_extractor_ndof_node,
        com_extractor_ndof_node,
        inertia_extractor_ndof_node,
        tcp_extractor_ndof_node,
        pas_frame_extractor_ndof_node,
        pseudo_angle_extractor_ndof_node,
        structure_digit_ndof_node,
        chain_active_twists,
        chain_passive_twists,
    ]


def generate_launch_description():
    # 1. Base directory - live synthesis YAML folder
    default_data_dir = os.path.expanduser(
        "~/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml/"
    )

    data_dir_arg = DeclareLaunchArgument(
        "data_dir",
        default_value=default_data_dir,
        description="Base directory where synthesis YAML files are written (live folder).",
    )

    # 2. Xacro path (can be absolute or relative to smm_synthesis share)
    xacro_path_arg = DeclareLaunchArgument(
        "xacro_path",
        default_value="urdf/6dof/smm_structure_anatomy_assembly_6dof.xacro",
        description=(
            "Absolute or package-relative path to the SMM structure xacro. "
            "DOF is inferred from the filename pattern '*_Xdof.xacro'."
        ),
    )

    # 3. Output filenames (within data_dir)
    frame_output_file_arg = DeclareLaunchArgument(
        "frame_output_file",
        default_value="gsai0.yaml",
        description="Filename for active joint frames (gsai0)",
    )
    com_output_file_arg = DeclareLaunchArgument(
        "com_output_file",
        default_value="gsli0.yaml",
        description="Filename for active CoM frames (gsli0)",
    )
    inertia_output_file_arg = DeclareLaunchArgument(
        "inertia_output_file",
        default_value="Mscomi0.yaml",
        description="Filename for spatial inertia tensor (Mscomi0)",
    )
    tcp_output_file_arg = DeclareLaunchArgument(
        "tcp_output_file",
        default_value="gst0.yaml",
        description="Filename for spatial TCP frame (gst0)",
    )
    act_twist_output_file_arg = DeclareLaunchArgument(
        "act_twist_output_file",
        default_value="xi_ai_anat.yaml",
        description="Filename for spatial active twists (xi_ai_anat)",
    )
    pas_frame_output_file_arg = DeclareLaunchArgument(
        "pas_frame_output_file",
        default_value="gspj0.yaml",
        description="Filename for passive joint frames (gspj0)",
    )
    pas_twist_output_file_arg = DeclareLaunchArgument(
        "pas_twist_output_file",
        default_value="xi_pj_anat.yaml",
        description="Filename for spatial passive twists (xi_pj_anat)",
    )
    pseudo_angle_output_file_arg = DeclareLaunchArgument(
        "pseudo_angle_output_file",
        default_value="q_pj_anat.yaml",
        description="Filename for pseudo joint angles (q_pj_anat)",
    )

    opaque = OpaqueFunction(function=launch_setup)

    return LaunchDescription(
        [
            data_dir_arg,
            xacro_path_arg,
            frame_output_file_arg,
            com_output_file_arg,
            inertia_output_file_arg,
            tcp_output_file_arg,
            act_twist_output_file_arg,
            pas_frame_output_file_arg,
            pas_twist_output_file_arg,
            pseudo_angle_output_file_arg,
            opaque,
        ]
    )
