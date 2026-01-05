#include <memory>
#include <string>
#include <fstream>
#include <iostream>
#include <map>
#include <vector>
#include <algorithm>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"

// Your screw-theory core
#include "smm_screws/core/ScrewsKinematics.h"

// HOW TO USE (example):
// ros2 run smm_synthesis twist_extractor_screws_ndof \
//   --ros-args \
//     -p input_file:=/home/nikos/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/ndof/yaml/gsai0.yaml \
//     -p output_file:=/home/nikos/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/ndof/yaml/xi_ai_anat.yaml

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("twist_extractor_screws_ndof");

  // --- Parameters: input + output YAML ---
  node->declare_parameter<std::string>("input_file", "");
  std::string input_path;
  node->get_parameter("input_file", input_path);

  if (input_path.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "No 'input_file' parameter provided. Use --ros-args -p input_file:=/full/path/gsai0.yaml");
    rclcpp::shutdown();
    return 1;
  }

  node->declare_parameter<std::string>("output_file", "");
  std::string output_path;
  node->get_parameter("output_file", output_path);

  if (output_path.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "No 'output_file' parameter provided. Use --ros-args -p output_file:=/full/path/xi_ai_anat.yaml");
    rclcpp::shutdown();
    return 1;
  }

  // --- Load input YAML with active frames ---
  YAML::Node root;
  try {
    root = YAML::LoadFile(input_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to load YAML file '%s': %s", input_path.c_str(), e.what());
    rclcpp::shutdown();
    return 1;
  }

  // --- Collect all 'gsa*' frame keys (gsa00, gsa10, gsa20, gsa30, ...) ---
  std::vector<std::string> gsa_keys;

  for (auto it = root.begin(); it != root.end(); ++it) {
    const std::string key = it->first.as<std::string>();
    if (key.rfind("gsa", 0) == 0) {  // starts with "gsa"
      gsa_keys.push_back(key);
    }
  }

  if (gsa_keys.empty()) {
    RCLCPP_ERROR(node->get_logger(),
                 "No 'gsa*' entries found in YAML '%s'. Nothing to do.",
                 input_path.c_str());
    rclcpp::shutdown();
    return 1;
  }

  std::sort(gsa_keys.begin(), gsa_keys.end());

  // Optional: limit between 3 and 6 DOF if you want to be strict
  if (gsa_keys.size() < 3 || gsa_keys.size() > 6) {
    RCLCPP_WARN(
      node->get_logger(),
      "Found %zu 'gsa*' frames. Expected between 3 and 6 for SMM. Proceeding anyway.",
      gsa_keys.size());
  }

  // --- Screw kinematics core ---
  ScrewsKinematics kin;

  // Store twists
  std::map<std::string, Eigen::Matrix<float, 6, 1>> twists;

  // --- For each gsa frame: build a twist ---
  for (std::size_t idx = 0; idx < gsa_keys.size(); ++idx) {
    const std::string & frame_name = gsa_keys[idx];

    if (!root[frame_name]) {
      RCLCPP_WARN(
        node->get_logger(),
        "Frame '%s' not found in input YAML '%s'. Skipping.",
        frame_name.c_str(), input_path.c_str());
      continue;
    }

    const YAML::Node & values = root[frame_name];
    if (!values.IsSequence() || values.size() != 16) {
      RCLCPP_WARN(
        node->get_logger(),
        "Frame '%s' has invalid matrix format (need 16 elements). Skipping.",
        frame_name.c_str());
      continue;
    }

    // 1) Parse 4x4 T
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 16; ++i) {
      T(i / 4, i % 4) = values[i].as<double>();
    }

    // 2) q = translation
    Eigen::Vector3d q_d = T.block<3,1>(0, 3);

    // 3) omega = axis direction (space frame)
    Eigen::Vector3d omega_d;
    if (frame_name == "gsa00" || idx == 0) {
      // Base joint: use Z axis of that frame
      omega_d = T.block<3,1>(0, 2);
    } else {
      // Other joints: use X axis of that frame (Dynamixel-style)
      omega_d = T.block<3,1>(0, 0);
    }

    // Cast to float for ScrewsKinematics
    Eigen::Vector3f q     = q_d.cast<float>();
    Eigen::Vector3f omega = omega_d.cast<float>();

    Eigen::Matrix<float, 6, 1> twist = kin.createTwist(omega, q);

    // Twist key: xi_a<i>_0  (0-based index)
    std::string twist_key = "xi_a" + std::to_string(idx) + "_0";
    twists[twist_key] = twist;

    std::cout << "\nFrame: " << frame_name << " → " << twist_key;
    std::cout << "\nTwist: " << twist.transpose() << "\n";
  }

  // --- Save twists to YAML ---
  YAML::Emitter out;
  out << YAML::BeginMap;
  for (const auto & pair : twists) {
    out << YAML::Key << pair.first << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < 6; ++i) {
      out << pair.second(i);
    }
    out << YAML::EndSeq;
  }
  out << YAML::EndMap;

  std::ofstream fout(output_path);
  if (!fout.is_open()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to open output YAML '%s' for writing.", output_path.c_str());
    rclcpp::shutdown();
    return 1;
  }
  fout << out.c_str();
  fout.close();

  RCLCPP_INFO(node->get_logger(), "Twists saved to: %s", output_path.c_str());

  rclcpp::shutdown();
  return 0;
}
