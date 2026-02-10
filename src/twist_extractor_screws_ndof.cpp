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
#include "smm_screws/core/ScrewsMain.h"   // or ScrewsKinematics if you prefer

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

  // --- Collect all 'gsa*' frame keys (gsa00, gsa10, gsa20, ...) ---
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

  const int dof = static_cast<int>(gsa_keys.size());  // <-- NO '-1' HERE

  if (dof < 3 || dof > 6) {
    RCLCPP_WARN(
      node->get_logger(),
      "Found %zu 'gsa*' frames → %d joints. Expected 3..6. Proceeding anyway.",
      gsa_keys.size(), dof);
  }

  ScrewsMain screws;
  std::map<std::string, Eigen::Matrix<float, 6, 1>> twists;

  for (int i = 0; i < dof; ++i) {
    const std::string & frame_name = gsa_keys[static_cast<std::size_t>(i)];

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

    // --- Parse 4x4 transform ---
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int k = 0; k < 16; ++k) {
      T(k / 4, k % 4) = values[k].as<double>();
    }

    Eigen::Vector3d q_d = T.block<3,1>(0, 3);
    Eigen::Vector3d omega_d;

    // Exact axis selection based on known joint types / frame names
    if (frame_name == "gsa00") {
      // Base stepper joint: axis = local Z
      omega_d = T.block<3,1>(0, 2);
    } else if (frame_name == "gsa10" || frame_name == "gsa20") {
      // First two DXL joints: axis = local X
      omega_d = T.block<3,1>(0, 0);
    } else if (frame_name == "gsa30" || frame_name == "gsa40" || frame_name == "gsa50") {
      // Wrist joints: axis = local Y
      omega_d = T.block<3,1>(0, 1);
    } else {
      // Unknown naming: you can add more patterns if needed
      RCLCPP_WARN(
        node->get_logger(),
        "Frame '%s': unknown joint axis pattern. Skipping.",
        frame_name.c_str());
      continue;
    }

    Eigen::Vector3f q     = q_d.cast<float>();
    Eigen::Vector3f omega = omega_d.cast<float>();

    Eigen::Matrix<float, 6, 1> twist = screws.createTwist(omega, q);

    // Derive joint index from frame name: "gsaXY" → X is joint index
    // With names gsa00, gsa10, gsa20, ... this yields 0,1,2,3,4,5
    int joint_idx = frame_name[3] - '0';

    std::string twist_key = "xi_a" + std::to_string(joint_idx) + "_0";
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
