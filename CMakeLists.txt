cmake_minimum_required(VERSION 3.8)
project(smm_synthesis)

# Use C++17 (ROS 2 default on most distros)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# --------------------------
# Dependencies (ament)
# --------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Core ROS2 + msgs
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# TF / URDF / KDL
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)

# YAML + your screws library
find_package(yaml-cpp REQUIRED)
find_package(smm_screws REQUIRED)

# --------------------------
# Include directories
# --------------------------
include_directories(
  include
  ${YAML_CPP_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${smm_screws_INCLUDE_DIRS}
)

# --------------------------
# Executables
# --------------------------

# 1) frame_extractor_kdl
add_executable(frame_extractor_kdl src/frame_extractor_kdl.cpp)
target_link_libraries(frame_extractor_kdl
  PUBLIC
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    kdl_parser::kdl_parser
    urdf::urdf
    ament_index_cpp::ament_index_cpp
    yaml-cpp
    ${orocos_kdl_LIBRARIES}     # or orocos_kdl::orocos_kdl if your install exports it
)
install(TARGETS frame_extractor_kdl
  DESTINATION lib/${PROJECT_NAME}
)

# 2) com_extractor_kdl
add_executable(com_extractor_kdl src/com_extractor_kdl.cpp)

target_link_libraries(com_extractor_kdl
  PUBLIC
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    tf2::tf2
    tf2_ros::tf2_ros
    tf2_geometry_msgs::tf2_geometry_msgs
    kdl_parser::kdl_parser
    urdf::urdf
    yaml-cpp
)

install(TARGETS com_extractor_kdl
  DESTINATION lib/${PROJECT_NAME})


# 3) inertia_extractor_kdl
add_executable(inertia_extractor_kdl src/inertia_extractor_kdl.cpp)

target_link_libraries(inertia_extractor_kdl
  PUBLIC
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    kdl_parser::kdl_parser
    urdf::urdf
    ament_index_cpp::ament_index_cpp
    yaml-cpp
    ${orocos_kdl_LIBRARIES}
)

install(TARGETS inertia_extractor_kdl
  DESTINATION lib/${PROJECT_NAME})

# 4) tcp_extractor_kdl
add_executable(tcp_extractor_kdl src/tcp_extractor_kdl.cpp)
target_link_libraries(tcp_extractor_kdl
  PUBLIC
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    tf2::tf2
    tf2_ros::tf2_ros
    tf2_geometry_msgs::tf2_geometry_msgs
    kdl_parser::kdl_parser
    urdf::urdf
    yaml-cpp
)

# 5) twist_extractor_screws (uses smm_screws)
add_executable(twist_extractor_screws src/twist_extractor_screws.cpp)

target_link_libraries(twist_extractor_screws
  PUBLIC
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    yaml-cpp
    ${smm_screws_LIBRARIES}
)

install(TARGETS twist_extractor_screws
  DESTINATION lib/${PROJECT_NAME})


# 6) passive_frame_extractor_kdl
add_executable(passive_frame_extractor_kdl src/passive_frame_extractor_kdl.cpp)

target_link_libraries(passive_frame_extractor_kdl
  PUBLIC
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    kdl_parser::kdl_parser
    urdf::urdf
    yaml-cpp
    ${orocos_kdl_LIBRARIES}   # or orocos_kdl::orocos_kdl if available
)

install(TARGETS passive_frame_extractor_kdl
  DESTINATION lib/${PROJECT_NAME})

# 7) passive_twist_extractor_screws (uses smm_screws)
add_executable(passive_twist_extractor_screws src/passive_twist_extractor_screws.cpp)

target_link_libraries(passive_twist_extractor_screws
  PUBLIC
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    yaml-cpp
    ${smm_screws_LIBRARIES} 
)

# 8) pseudo_angle_extractor
add_executable(pseudo_angle_extractor
  src/pseudo_angle_extractor.cpp
)

target_link_libraries(pseudo_angle_extractor
  PUBLIC
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    yaml-cpp
)

install(TARGETS pseudo_angle_extractor
  DESTINATION lib/${PROJECT_NAME}
)

# 9) structure_digit_setter
add_executable(structure_digit_setter
  src/structure_digit_setter.cpp
)

target_link_libraries(structure_digit_setter
  PUBLIC
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    yaml-cpp
)

install(TARGETS structure_digit_setter
  DESTINATION lib/${PROJECT_NAME}
)

# 10) generic (3-6 dof) active frame extractor
add_executable(active_frames_extractor_ndof 
  src/active_frames_extractor_ndof.cpp
)

target_link_libraries(active_frames_extractor_ndof
  PUBLIC
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    kdl_parser::kdl_parser
    urdf::urdf
    yaml-cpp
    ${orocos_kdl_LIBRARIES}   # or orocos_kdl::orocos_kdl if available
)

install(TARGETS active_frames_extractor_ndof 
  DESTINATION lib/${PROJECT_NAME}
)

# 11) generic (3-6 dof) CoM frame extractor
add_executable(com_extractor_kdl_ndof 
  src/com_extractor_kdl_ndof.cpp
)

target_link_libraries(com_extractor_kdl_ndof
  PUBLIC
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    kdl_parser::kdl_parser
    urdf::urdf
    yaml-cpp
    ${orocos_kdl_LIBRARIES}   # or orocos_kdl::orocos_kdl if available
)

install(TARGETS com_extractor_kdl_ndof
  DESTINATION lib/${PROJECT_NAME})

# 12) generic (3-6 dof) inertia extractor + smm_screws core method 
add_executable(inertia_extractor_kdl_ndof
  src/inertia_extractor_kdl_ndof.cpp
)

target_link_libraries(inertia_extractor_kdl_ndof
  PUBLIC
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    kdl_parser::kdl_parser
    urdf::urdf
    yaml-cpp
    ${smm_screws_LIBRARIES}
    ${orocos_kdl_LIBRARIES}   # or orocos_kdl::orocos_kdl if available
)

install(TARGETS inertia_extractor_kdl_ndof
  DESTINATION lib/${PROJECT_NAME})

# 13) generic (3-6 dof) TCP frame extractor
add_executable(tcp_extractor_kdl_ndof 
  src/tcp_extractor_kdl_ndof.cpp
)

target_link_libraries(tcp_extractor_kdl_ndof
PUBLIC
  rclcpp::rclcpp
  ament_index_cpp::ament_index_cpp
  kdl_parser::kdl_parser
  urdf::urdf
  yaml-cpp
  ${orocos_kdl_LIBRARIES}   # or orocos_kdl::orocos_kdl if available
)

install(TARGETS tcp_extractor_kdl_ndof
DESTINATION lib/${PROJECT_NAME})

# 14) twist_extractor_screws for ndof (uses smm_screws)
add_executable(twist_extractor_screws_ndof src/twist_extractor_screws_ndof.cpp)

target_link_libraries(twist_extractor_screws_ndof
  PUBLIC
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    yaml-cpp
    ${smm_screws_LIBRARIES}
)

install(TARGETS twist_extractor_screws_ndof
  DESTINATION lib/${PROJECT_NAME})

# 15) passive_frame_extractor_kdl for ndof
add_executable(passive_frame_extractor_kdl_ndof src/passive_frame_extractor_kdl_ndof.cpp)

target_link_libraries(passive_frame_extractor_kdl_ndof
  PUBLIC
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    kdl_parser::kdl_parser
    urdf::urdf
    yaml-cpp
    ${orocos_kdl_LIBRARIES}   # or orocos_kdl::orocos_kdl if available
)

install(TARGETS passive_frame_extractor_kdl_ndof
  DESTINATION lib/${PROJECT_NAME})

# 16) passive_twist_extractor_screws for ndof (uses smm_screws)
add_executable(passive_twist_extractor_screws_ndof src/passive_twist_extractor_screws_ndof.cpp)

target_link_libraries(passive_twist_extractor_screws_ndof
  PUBLIC
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    yaml-cpp
    ${smm_screws_LIBRARIES} 
)

# 17) pseudo_angle_extractor for ndof
add_executable(pseudo_angle_extractor_ndof
  src/pseudo_angle_extractor_ndof.cpp
)

target_link_libraries(pseudo_angle_extractor_ndof
  PUBLIC
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    yaml-cpp
)

install(TARGETS pseudo_angle_extractor_ndof
  DESTINATION lib/${PROJECT_NAME}
)

# 18) structure_digit_setter generic case
add_executable(structure_digit_setter_ndof
  src/structure_digit_setter.cpp
)

target_link_libraries(structure_digit_setter_ndof
  PUBLIC
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    yaml-cpp
)

install(TARGETS structure_digit_setter_ndof
  DESTINATION lib/${PROJECT_NAME}
)

# --------------------------
# Install targets & resources
# --------------------------
install(TARGETS
  frame_extractor_kdl
  com_extractor_kdl
  inertia_extractor_kdl
  tcp_extractor_kdl
  twist_extractor_screws
  passive_frame_extractor_kdl
  passive_twist_extractor_screws
  pseudo_angle_extractor
  structure_digit_setter
  active_frames_extractor_ndof
  com_extractor_kdl_ndof
  inertia_extractor_kdl_ndof
  tcp_extractor_kdl_ndof
  twist_extractor_screws_ndof
  passive_frame_extractor_kdl_ndof
  passive_twist_extractor_screws_ndof
  pseudo_angle_extractor_ndof
  structure_digit_setter_ndof
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch / urdf / config / meshes if you have them
install(DIRECTORY
  launch
  urdf
  config
  meshes
  DESTINATION share/${PROJECT_NAME}
)

# --------------------------
# ament package
# --------------------------
ament_package()
