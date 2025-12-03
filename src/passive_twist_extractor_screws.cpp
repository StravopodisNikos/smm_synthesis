#include <memory>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <iostream>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "smm_screws/core/ScrewsKinematics.h"  // updated include path

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("passive_twist_extractor_screws");

  // ---- Get input YAML file name as parameter ----
  node->declare_parameter<std::string>("input_file", "");
  std::string input_file = node->get_parameter("input_file").as_string();
  if (input_file.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "No 'input_file' parameter provided. Use --ros-args -p input_file:=gspj0.yaml");
    rclcpp::shutdown();
    return 1;
  }

  // ---- Resolve package share dir ----
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

  // ---- Read structure parameters from assembly.yaml ----
  const std::string structure_file = share_dir + "/config/yaml/assembly.yaml";
  YAML::Node assembly;
  try {
    assembly = YAML::LoadFile(structure_file);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to load assembly YAML '%s': %s",
      structure_file.c_str(), e.what());
    rclcpp::shutdown();
    return 1;
  }

  std::map<std::string, std::string> frame_map = {
    {"s2", "metalink_1_pseudo1_b"},
    {"s3", "metalink_1_pseudo2_b"},
    {"s5", "metalink_2_pseudo1_b"},
    {"s6", "metalink_2_pseudo2_b"}
  };

  std::vector<std::string> active_frames;
  for (const auto & param : frame_map) {
    if (assembly[param.first] && assembly[param.first].as<int>() != 9) {
      active_frames.push_back(param.second);
    }
  }

  if (active_frames.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "No active passive joints found (s2, s3, s5, s6 all equal to 9).");
    rclcpp::shutdown();
    return 1;
  }

  // ---- Load input transforms (gspj0.yaml) ----
  const std::string input_path = share_dir + "/config/yaml/" + input_file;
  YAML::Node frame_data;
  try {
    frame_data = YAML::LoadFile(input_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to load input YAML '%s': %s",
      input_path.c_str(), e.what());
    rclcpp::shutdown();
    return 1;
  }

  ScrewsKinematics kin;

  // use float to align with ScrewsKinematics internals
  std::map<std::string, Eigen::Matrix<float, 6, 1>> twists;

  int index = 0;
  for (const auto & frame_name : active_frames) {
    if (!frame_data[frame_name]) {
      RCLCPP_WARN(
        node->get_logger(),
        "Frame '%s' not found in input file '%s'.",
        frame_name.c_str(), input_path.c_str());
      continue;
    }

    const YAML::Node & values = frame_data[frame_name];
    if (!values.IsSequence() || values.size() != 16) {
      RCLCPP_WARN(
        node->get_logger(),
        "Skipping '%s' due to invalid matrix format (need 16 elements).",
        frame_name.c_str());
      continue;
    }

    // 4x4 homogeneous transform
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 16; ++i) {
      T(i / 4, i % 4) = values[i].as<float>();
    }

    // For passive joints you used column 0 as axis (X) and column 3 as q
    Eigen::Vector3f omega = T.block<3, 1>(0, 0);
    Eigen::Vector3f q     = T.block<3, 1>(0, 3);

    Eigen::Matrix<float, 6, 1> twist = kin.createTwist(omega, q);

    std::string twist_key = "xi_p" + std::to_string(index++) + "_0";
    twists[twist_key] = twist;

    std::cout << "\nFrame: " << frame_name << " → " << twist_key << "\n";
    std::cout << "Twist: " << twist.transpose() << "\n";
  }

  // ---- Save to xi_pj_anat.yaml ----
  const std::string output_path = share_dir + "/config/yaml/xi_pj_anat.yaml";

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
      "Failed to open output file '%s' for writing.", output_path.c_str());
    rclcpp::shutdown();
    return 1;
  }
  fout << out.c_str();
  fout.close();

  RCLCPP_INFO(
    node->get_logger(),
    "Passive twists saved to: %s", output_path.c_str());

  rclcpp::shutdown();
  return 0;
}
