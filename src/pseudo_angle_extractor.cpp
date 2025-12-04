#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>

#include <yaml-cpp/yaml.h>
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pseudo_angle_extractor");

  // --- 1. Get output_file as FULL PATH from ROS 2 parameter ---
  node->declare_parameter<std::string>("output_file", "");
  std::string output_path = node->get_parameter("output_file").as_string();
  if (output_path.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "No 'output_file' parameter provided. Use --ros-args -p output_file:=/full/path/q_pj_anat.yaml");
    rclcpp::shutdown();
    return 1;
  }

  // --- 2. Locate assembly.yaml inside the smm_synthesis package ---
  std::string share_dir;
  try {
    share_dir = ament_index_cpp::get_package_share_directory("smm_synthesis");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to get package share directory for 'smm_synthesis': %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  const std::string input_path = share_dir + "/config/yaml/assembly.yaml";

  // --- 3. Load assembly.yaml ---
  YAML::Node root;
  try {
    root = YAML::LoadFile(input_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to load assembly.yaml ('%s'): %s",
      input_path.c_str(), e.what());
    rclcpp::shutdown();
    return 1;
  }

  // Extract s-parameters (assume they exist and are ints)
  int s2 = root["s2"].as<int>();
  int s3 = root["s3"].as<int>();
  int s5 = root["s5"].as<int>();
  int s6 = root["s6"].as<int>();

  // Extract all 4 angles
  std::vector<float> all_angles;
  try {
    all_angles = {
      root["pseudo1_angle"].as<float>(),
      root["pseudo2_angle"].as<float>(),
      root["pseudo3_angle"].as<float>(),
      root["pseudo4_angle"].as<float>()
    };
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to read pseudo*_angle entries from '%s': %s",
      input_path.c_str(), e.what());
    rclcpp::shutdown();
    return 1;
  }

  // Filter valid angles (non-9 structure digits)
  std::vector<float> filtered_angles;
  if (s2 != 9) filtered_angles.push_back(all_angles[0]);
  if (s3 != 9) filtered_angles.push_back(all_angles[1]);
  if (s5 != 9) filtered_angles.push_back(all_angles[2]);
  if (s6 != 9) filtered_angles.push_back(all_angles[3]);

  // --- 4. Emit YAML into the requested output_file path ---
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "pseudo_angles" << YAML::Value << YAML::BeginSeq;
  for (float angle : filtered_angles) {
    out << angle;
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;

  std::ofstream fout(output_path);
  if (!fout.is_open()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to open output file '%s' for writing.", output_path.c_str());
    rclcpp::shutdown();
    return 1;
  }
  fout << out.c_str();
  fout.close();

  RCLCPP_INFO(
    node->get_logger(),
    "Filtered pseudo angles saved to: %s",
    output_path.c_str());

  rclcpp::shutdown();
  return 0;
}
