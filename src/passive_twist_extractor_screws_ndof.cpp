#include <memory>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <iostream>
#include <chrono>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"

// Use ScrewsMain for twist creation
#include "smm_screws/core/ScrewsMain.h"

// HOW TO USE (in synthesis pipeline via master_synthesisNdof.launch.py):
//
//   master_synthesisNdof.launch.py should pass:
//
//     input_file    := <data_dir>/gspj0.yaml        (live passive frames)
//     output_file   := <data_dir>/xi_pj_anat.yaml   (live passive twists)
//     assembly_yaml := <pkg_share>/config/yaml/<Xdof>/assembly_<Xdof>.yaml (template)
//
// Manual example:
//
//   ros2 run smm_synthesis passive_twist_extractor_screws_ndof \
//     --ros-args \
//       -p input_file:=/full/path/gspj0.yaml \
//       -p output_file:=/full/path/xi_pj_anat.yaml \
//       -p assembly_yaml:=/full/path/assembly_6dof.yaml

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("passive_twist_extractor_screws_ndof");

  using namespace std::chrono_literals;

  // ---- 1) Get input YAML file name as parameter (full path, live folder) ----
  node->declare_parameter<std::string>("input_file", "");
  std::string input_file;
  node->get_parameter("input_file", input_file);

  if (input_file.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "No 'input_file' parameter provided. Use --ros-args -p input_file:=/full/path/gspj0.yaml");
    rclcpp::shutdown();
    return 1;
  }

  // ---- 2) Get output YAML file name as parameter (full path, live folder) ----
  node->declare_parameter<std::string>("output_file", "");
  std::string output_file;
  node->get_parameter("output_file", output_file);

  if (output_file.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "No 'output_file' parameter provided. Use --ros-args "
      "-p output_file:=/full/path/xi_pj_anat.yaml");
    rclcpp::shutdown();
    return 1;
  }

  // ---- 3) Assembly YAML: DOF-specific TEMPLATE, not live assembly.yaml ----
  node->declare_parameter<std::string>("assembly_yaml", "");
  std::string assembly_path;
  node->get_parameter("assembly_yaml", assembly_path);

  if (assembly_path.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Parameter 'assembly_yaml' is empty. "
      "This should point to the DOF-specific template, e.g. "
      "'<pkg_share>/config/yaml/6dof/assembly_6dof.yaml'.");
    rclcpp::shutdown();
    return 1;
  }

  YAML::Node assembly;
  try {
    assembly = YAML::LoadFile(assembly_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to load assembly YAML '%s': %s",
      assembly_path.c_str(), e.what());
    rclcpp::shutdown();
    return 1;
  }

  // ---- 4) Map structure digits → URDF link names for passive joints ----
  // Covers up to 3 metalinks (6 possible passive joints).
  std::vector<std::pair<std::string, std::string>> frame_map = {
    {"s2", "metalink_1_pseudo1_b"},
    {"s3", "metalink_1_pseudo2_b"},
    {"s5", "metalink_2_pseudo1_b"},
    {"s6", "metalink_2_pseudo2_b"},
    {"s8", "metalink_3_pseudo1_b"},
    {"s9", "metalink_3_pseudo2_b"},
  };

  std::vector<std::string> active_frames;
  for (const auto & param : frame_map) {
    const std::string & s_key     = param.first;
    const std::string & link_name = param.second;

    if (!assembly[s_key]) {
      RCLCPP_WARN(
        node->get_logger(),
        "Structure digit '%s' missing in assembly YAML '%s'; skipping '%s'.",
        s_key.c_str(), assembly_path.c_str(), link_name.c_str());
      continue;
    }

    int digit = assembly[s_key].as<int>();
    if (digit != 9) {
      active_frames.push_back(link_name);
    }
  }

  if (active_frames.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "No active passive joints found (all s2, s3, s5, s6, s8, s9 are 9 or missing).");
    rclcpp::shutdown();
    return 1;
  }

  // Give passive_frame_extractor_kdl_ndof a bit of time to write gspj0.yaml
  rclcpp::sleep_for(1s);

  // ---- 5) Load input transforms (gspj0.yaml from LIVE folder) ----
  const std::string input_path = input_file;  // already full path from launch

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

  RCLCPP_INFO(
    node->get_logger(),
    "Loaded passive frame YAML from: %s", input_path.c_str());

  // ---- 6) Use ScrewsMain to compute twists from transforms ----
  ScrewsMain screws;  // base screw-theory helper

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

    // 4x4 homogeneous transform (float)
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 16; ++i) {
      T(i / 4, i % 4) = values[i].as<float>();
    }

    // For passive joints:
    //   omega = X-axis of the frame (column 0)
    //   q     = position vector (column 3)
    Eigen::Vector3f omega = T.block<3, 1>(0, 0);
    Eigen::Vector3f q     = T.block<3, 1>(0, 3);

    Eigen::Matrix<float, 6, 1> twist = screws.createTwist(omega, q);

    std::string twist_key = "xi_p" + std::to_string(index++) + "_0";
    twists[twist_key] = twist;

    std::cout << "\nFrame: " << frame_name << " → " << twist_key << "\n";
    std::cout << "Twist: " << twist.transpose() << "\n";
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Active passive frames (from assembly '%s'): %zu, computed passive twists: %zu",
    assembly_path.c_str(), active_frames.size(), twists.size());

  // ---- 7) Save to output_file (full path from param, live folder) ----
  const std::string output_path = output_file;

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
