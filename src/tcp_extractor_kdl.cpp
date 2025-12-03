#include <memory>
#include <string>
#include <fstream>
#include <iostream>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("tcp_extractor_kdl");

  using namespace std::chrono_literals;

  // --- Get output filename parameter ---
  node->declare_parameter<std::string>("output_file", "");
  std::string output_filename;
  node->get_parameter("output_file", output_filename);

  if (output_filename.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "No 'output_file' parameter given. Use --ros-args -p output_file:=gst0.yaml");
    return 1;
  }

  // --- Get URDF from 'robot_description' parameter ---
  node->declare_parameter<std::string>("robot_description", "");
  std::string robot_description;
  node->get_parameter("robot_description", robot_description);

  if (robot_description.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Parameter 'robot_description' is empty. Make sure it is set in your launch file.");
    return 1;
  }

  urdf::Model model;
  if (!model.initString(robot_description)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to parse URDF from 'robot_description'.");
    return 1;
  }

  // --- Build KDL tree ---
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to parse KDL tree from URDF.");
    return 1;
  }

  const std::string base_link = kdl_tree.getRootSegment()->first;
  const std::string tip_link  = "massage_tool";

  KDL::Chain chain;
  if (!kdl_tree.getChain(base_link, tip_link, chain)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to extract chain from %s to %s",
      base_link.c_str(), tip_link.c_str());
    return 1;
  }

  // --- Add fixed frame from massage_tool to tcp ---
  KDL::Frame massage_tool_to_tcp_frame(
    KDL::Rotation::Quaternion(0.0, 0.0, 0.0, 1.0),   // identity rotation
    KDL::Vector(0.00, 0.00, 0.065)                   // your TCP offset
  );
  KDL::Segment fixed_tcp_segment(
    "tcp",
    KDL::Joint(KDL::Joint::None),
    massage_tool_to_tcp_frame
  );
  chain.addSegment(fixed_tcp_segment);

  // --- Compute FK with all joints at zero ---
  KDL::Frame world_to_tcp = KDL::Frame::Identity();
  for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
    const KDL::Segment & segment = chain.getSegment(i);
    world_to_tcp = world_to_tcp * segment.pose(0.0);
  }

  // --- Convert to Eigen 4x4 ---
  Eigen::Matrix4d tf_matrix = Eigen::Matrix4d::Identity();
  for (int i = 0; i < 3; ++i) {
    tf_matrix(i, 3) = world_to_tcp.p[i];
    for (int j = 0; j < 3; ++j) {
      tf_matrix(i, j) = world_to_tcp.M(i, j);
    }
  }

  std::cout << "\nTransform from world to TCP (4x4 matrix):\n"
            << tf_matrix << "\n";

  // --- Build save path using ament_index instead of ros::package ---
  std::string share_dir;
  try {
    share_dir = ament_index_cpp::get_package_share_directory("smm_synthesis");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Could not get package share directory for 'smm_synthesis': %s", e.what());
    return 1;
  }

  std::string save_path = share_dir + "/config/yaml/" + output_filename;

  // --- Save to YAML ---
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "gst0" << YAML::Value << YAML::BeginSeq;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      out << tf_matrix(i, j);
    }
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;

  std::ofstream fout(save_path);
  if (!fout.is_open()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to open file for writing: %s", save_path.c_str());
    return 1;
  }
  fout << out.c_str();
  fout.close();

  RCLCPP_INFO(node->get_logger(), "TCP frame saved to: %s", save_path.c_str());

  rclcpp::shutdown();
  return 0;
}
