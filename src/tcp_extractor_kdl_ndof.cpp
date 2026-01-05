#include <memory>
#include <string>
#include <fstream>
#include <iostream>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <urdf/model.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("tcp_extractor_kdl");

  using namespace std::chrono_literals;

  // --- Parameters ---

  // 1) output_file: FULL PATH to YAML
  node->declare_parameter<std::string>("output_file", "");
  std::string output_filename;
  node->get_parameter("output_file", output_filename);
  if (output_filename.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "No 'output_file' parameter given. Use --ros-args -p output_file:=/path/to/gst0.yaml");
    return 1;
  }

  // 2) robot_description: URDF string from xacro
  node->declare_parameter<std::string>("robot_description", "");
  std::string robot_description;
  node->get_parameter("robot_description", robot_description);
  if (robot_description.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Parameter 'robot_description' is empty. Make sure it is set in your launch file.");
    return 1;
  }

  // 3) base_link: optional, default = KDL tree root
  node->declare_parameter<std::string>("base_link", "");
  std::string base_link_param;
  node->get_parameter("base_link", base_link_param);

  // 4) tip_link: optional, default = "tcp" (as in your xacro)
  node->declare_parameter<std::string>("tip_link", "tcp");
  std::string tip_link;
  node->get_parameter("tip_link", tip_link);
  if (tip_link.empty()) {
    RCLCPP_ERROR(node->get_logger(),
                 "Parameter 'tip_link' is empty. Set it to your TCP link (e.g. 'tcp').");
    return 1;
  }

  // tiny sleep so launch param machinery settles (not strictly needed but harmless)
  rclcpp::sleep_for(1s);

  // --- Build URDF model from robot_description ---
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

  // Decide base link
  std::string base_link;
  if (!base_link_param.empty()) {
    base_link = base_link_param;
    RCLCPP_INFO(node->get_logger(),
                "Using base_link from parameter: '%s'", base_link.c_str());
  } else {
    base_link = kdl_tree.getRootSegment()->first;
    RCLCPP_INFO(node->get_logger(),
                "Parameter 'base_link' not set. Using KDL root: '%s'",
                base_link.c_str());
  }

  RCLCPP_INFO(node->get_logger(),
              "Extracting TCP from base '%s' to tip '%s'",
              base_link.c_str(), tip_link.c_str());

  // --- Build chain base_link → tip_link ---
  KDL::Chain chain;
  if (!kdl_tree.getChain(base_link, tip_link, chain)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to extract chain from '%s' to '%s'",
      base_link.c_str(), tip_link.c_str());
    return 1;
  }

  // --- FK with all joints at zero ---
  KDL::Frame base_to_tcp = KDL::Frame::Identity();
  for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
    const KDL::Segment & segment = chain.getSegment(i);
    // pose(q) — here we evaluate at joint position = 0.0 for all joints
    base_to_tcp = base_to_tcp * segment.pose(0.0);
  }

  // --- Convert to Eigen 4x4 ---
  Eigen::Matrix4d tf_matrix = Eigen::Matrix4d::Identity();
  for (int i = 0; i < 3; ++i) {
    tf_matrix(i, 3) = base_to_tcp.p[i];
    for (int j = 0; j < 3; ++j) {
      tf_matrix(i, j) = base_to_tcp.M(i, j);
    }
  }

  std::cout << "\nTransform from '" << base_link
            << "' to '" << tip_link << "' (4x4 matrix):\n"
            << tf_matrix << "\n";

  // --- Save to YAML (key: gst0) ---
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

  std::ofstream fout(output_filename);
  if (!fout.is_open()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to open file for writing: %s", output_filename.c_str());
    return 1;
  }
  fout << out.c_str();
  fout.close();

  RCLCPP_INFO(
    node->get_logger(),
    "TCP frame saved to: %s", output_filename.c_str());

  rclcpp::shutdown();
  return 0;
}
