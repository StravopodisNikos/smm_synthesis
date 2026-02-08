#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>

#include <yaml-cpp/yaml.h>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pseudo_angle_extractor_ndof");

  // --- 1. Get output_file as FULL PATH from ROS 2 parameter ---
  node->declare_parameter<std::string>("output_file", "");
  std::string output_path = node->get_parameter("output_file").as_string();
  if (output_path.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "No 'output_file' parameter provided. Use --ros-args "
      "-p output_file:=/full/path/q_pj_anat.yaml");
    rclcpp::shutdown();
    return 1;
  }

  // --- 2. Get assembly_yaml parameter (DOF-specific TEMPLATE) ---
  // e.g. <pkg_share>/config/yaml/3dof/assembly_3dof.yaml
  //      <pkg_share>/config/yaml/6dof/assembly_6dof.yaml
  node->declare_parameter<std::string>("assembly_yaml", "");
  std::string assembly_path = node->get_parameter("assembly_yaml").as_string();
  if (assembly_path.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "No 'assembly_yaml' parameter provided. This must point to the "
      "DOF-specific assembly template (e.g. .../assembly_6dof.yaml).");
    rclcpp::shutdown();
    return 1;
  }

  // --- 3. Load assembly template YAML ---
  YAML::Node root;
  try {
    root = YAML::LoadFile(assembly_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to load assembly template ('%s'): %s",
      assembly_path.c_str(), e.what());
    rclcpp::shutdown();
    return 1;
  }

  // --- 4. Generic mapping: structure digit -> angle key ---
  // This covers up to 3 metalinks (6 pseudo joints):
  //   - metalink 1: s2, s3
  //   - metalink 2: s5, s6
  //   - metalink 3: s8, s9
  struct PseudoSpec
  {
    std::string s_key;       // e.g., "s2"
    std::string angle_key;   // e.g., "pseudo1_angle"
  };

  std::vector<PseudoSpec> specs = {
    {"s2", "pseudo1_angle"},
    {"s3", "pseudo2_angle"},
    {"s5", "pseudo3_angle"},
    {"s6", "pseudo4_angle"},
    {"s8", "pseudo5_angle"},
    {"s9", "pseudo6_angle"},
  };

  std::vector<float> filtered_angles;

  for (const auto & spec : specs) {
    // 4.1 Check that the structure digit exists
    if (!root[spec.s_key]) {
      RCLCPP_WARN(
        node->get_logger(),
        "Structure digit '%s' not found in assembly template '%s', "
        "skipping corresponding angle '%s'.",
        spec.s_key.c_str(), assembly_path.c_str(), spec.angle_key.c_str());
      continue;
    }

    int s_val = root[spec.s_key].as<int>();

    // If s == 9, pseudo does not exist → skip
    if (s_val == 9) {
      continue;
    }

    // 4.2 Pseudo exists → read its angle key if present
    if (!root[spec.angle_key]) {
      RCLCPP_WARN(
        node->get_logger(),
        "Angle key '%s' not found in assembly template '%s', even though %s != 9. Skipping.",
        spec.angle_key.c_str(), assembly_path.c_str(), spec.s_key.c_str());
      continue;
    }

    try {
      float angle = root[spec.angle_key].as<float>();
      filtered_angles.push_back(angle);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Failed to parse '%s' as float in '%s': %s",
        spec.angle_key.c_str(), assembly_path.c_str(), e.what());
      rclcpp::shutdown();
      return 1;
    }
  }

  if (filtered_angles.empty()) {
    RCLCPP_WARN(
      node->get_logger(),
      "No valid pseudo angles found (all digits 9 or angle keys missing). "
      "Writing an empty 'pseudo_angles' list.");
  }

  // --- 5. Emit YAML into the requested output_file path ---
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
    "Filtered pseudo angles (%zu) saved to: %s",
    filtered_angles.size(), output_path.c_str());

  rclcpp::shutdown();
  return 0;
}
