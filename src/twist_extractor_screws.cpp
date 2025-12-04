#include <memory>
#include <string>
#include <fstream>
#include <iostream>
#include <map>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"

// Include the Screws library (ROS2 version)
#include "smm_screws/core/ScrewsKinematics.h"

// Run for test (ROS2):
// $ ros2 run smm_synthesis twist_extractor_screws --ros-args \
//     -p input_file:=/home/nikos/ros2_ws/src/smm_data/synthesis/yaml/gsai0.yaml \
//     -p output_file:=/home/nikos/ros2_ws/src/smm_data/synthesis/yaml/xi_ai_anat.yaml

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("twist_extractor_screws");

  // --- Get YAML file name from ROS 2 parameter ---
  node->declare_parameter<std::string>("input_file", "");
  std::string yaml_file;
  node->get_parameter("input_file", yaml_file);

  if (yaml_file.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "No 'input_file' parameter provided. Use --ros-args -p input_file:=gsai0.yaml");
    rclcpp::shutdown();
    return 1;
  }

  // --- Get output file name from ROS 2 parameter (full path) ---
  node->declare_parameter<std::string>("output_file", "");
  std::string output_file;
  node->get_parameter("output_file", output_file);

  if (output_file.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "No 'output_file' parameter provided. Use --ros-args "
      "-p output_file:=/full/path/to/xi_ai_anat.yaml");
    rclcpp::shutdown();
    return 1;
  }

  std::string input_path  = yaml_file;
  std::string output_path = output_file;

  // --- Load input YAML ---
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

  ScrewsKinematics kin;

  // store twists as float-based Eigen (to match ScrewsKinematics)
  std::map<std::string, Eigen::Matrix<float, 6, 1>> twists;

  // Map frame name in YAML -> twist key name in output YAML
  std::map<std::string, std::string> twist_name_map = {
    {"gsa00", "xi_a0_0"},
    {"gsa10", "xi_a1_0"},
    {"gsa20", "xi_a2_0"}
  };

  for (const auto & pair : twist_name_map) {
    const std::string & frame_name = pair.first;
    const std::string & twist_key  = pair.second;

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

    // --- Convert YAML into Eigen 4x4 (double for parsing) ---
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 16; ++i) {
      T(i / 4, i % 4) = values[i].as<double>();
    }

    // Extract q (position)
    Eigen::Vector3d q_d = T.block<3,1>(0, 3);

    // Extract omega (rotation axis)
    Eigen::Vector3d omega_d;
    if (frame_name == "gsa00") {
      omega_d = T.block<3,1>(0, 2);  // Z axis for base_link
    } else {
      omega_d = T.block<3,1>(0, 0);  // X axis for the other joints
    }

    // Cast to float to match ScrewsKinematics::createTwist signature
    Eigen::Vector3f q     = q_d.cast<float>();
    Eigen::Vector3f omega = omega_d.cast<float>();

    Eigen::Matrix<float, 6, 1> twist = kin.createTwist(omega, q);
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
