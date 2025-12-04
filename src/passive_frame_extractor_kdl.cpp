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

#include <urdf/model.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

// How to run standalone:
/*
ros2 run smm_synthesis passive_frame_extractor_kdl \
  --ros-args \
  -p output_file:=/home/nikos/ros2_ws/src/smm_data/synthesis/yaml/gspj0.yaml \
  -p robot_description:="$(xacro /home/nikos/ros2_ws/install/smm_synthesis/share/smm_synthesis/urdf/smm_structure_anatomy_assembly.xacro)"

*/

Eigen::Matrix4d KDLFrameToEigen(const KDL::Frame & frame)
{
  Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      mat(i, j) = frame.M(i, j);
    }
  }
  mat(0, 3) = frame.p.x();
  mat(1, 3) = frame.p.y();
  mat(2, 3) = frame.p.z();
  return mat;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("passive_frame_extractor_kdl");

  // --- Parameters ---
  node->declare_parameter<std::string>("output_file", "");
  std::string output_filename = node->get_parameter("output_file").as_string();
  if (output_filename.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "No 'output_file' parameter given. Use --ros-args -p output_file:=gspj0.yaml");
    rclcpp::shutdown();
    return 1;
  }

  // --- Read structure digits from assembly.yaml (like ROS1 + rosparam) ---
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

  int s2 = assembly["s2"].as<int>();
  int s3 = assembly["s3"].as<int>();
  int s5 = assembly["s5"].as<int>();
  int s6 = assembly["s6"].as<int>();

  if (s2 == 9 && s3 == 9 && s5 == 9 && s6 == 9) {
    RCLCPP_WARN(
      node->get_logger(),
      "All structure digits (s2,s3,s5,s6) are 9 → no passive frames will be extracted.");
  }

  // URDF string from parameter 'robot_description'
  node->declare_parameter<std::string>("robot_description", "");
  std::string urdf_xml;
  node->get_parameter("robot_description", urdf_xml);
  if (urdf_xml.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Parameter 'robot_description' is empty. Pass URDF XML via launch.");
    rclcpp::shutdown();
    return 1;
  }

  urdf::Model model;
  if (!model.initString(urdf_xml)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to parse URDF from 'robot_description' parameter.");
    rclcpp::shutdown();
    return 1;
  }

  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to construct KDL tree from URDF.");
    rclcpp::shutdown();
    return 1;
  }

  const std::string root_name = "base_link";
  RCLCPP_INFO(
    node->get_logger(),
    "Setting root frame to '%s' with rotz(-pi) compensation", root_name.c_str());

  // decide which passive frames to extract
  std::vector<std::string> frames_to_extract;
  if (s2 != 9) frames_to_extract.push_back("metalink_1_pseudo1_b");
  if (s3 != 9) frames_to_extract.push_back("metalink_1_pseudo2_b");
  if (s5 != 9) frames_to_extract.push_back("metalink_2_pseudo1_b");
  if (s6 != 9) frames_to_extract.push_back("metalink_2_pseudo2_b");

  std::map<std::string, Eigen::Matrix4d> extracted_transforms;

  // rotz(-pi)
  Eigen::Matrix4d Rz_pi = Eigen::Matrix4d::Identity();
  Rz_pi(0, 0) = -1.0;
  Rz_pi(1, 1) = -1.0;

  for (const auto & frame_name : frames_to_extract) {
    KDL::Chain chain;
    if (!kdl_tree.getChain(root_name, frame_name, chain)) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Failed to extract chain from '%s' to '%s'",
        root_name.c_str(), frame_name.c_str());
      continue;
    }

    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::JntArray q(chain.getNrOfJoints());
    for (unsigned int i = 0; i < chain.getNrOfJoints(); ++i) {
      q(i) = 0.0;
    }

    KDL::Frame tf;
    if (fk_solver.JntToCart(q, tf) >= 0) {
      Eigen::Matrix4d tf_eigen = KDLFrameToEigen(tf);
      Eigen::Matrix4d transformed = Rz_pi * tf_eigen;
      extracted_transforms[frame_name] = transformed;

      std::cout << "\nSpatial Transform (" << root_name
                << " -> " << frame_name << ") with rotz(-pi):\n"
                << transformed << "\n";
    } else {
      RCLCPP_WARN(
        node->get_logger(),
        "FK failed for frame '%s'", frame_name.c_str());
    }
  }

  // Write YAML
  YAML::Emitter out;
  out << YAML::BeginMap;
  for (const auto & entry : extracted_transforms) {
    out << YAML::Key << entry.first << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        out << entry.second(i, j);
      }
    }
    out << YAML::EndSeq;
  }
  out << YAML::EndMap;

  std::ofstream fout(output_filename);
  if (!fout.is_open()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to open output file '%s' for writing.", output_filename.c_str());
    rclcpp::shutdown();
    return 1;
  }
  fout << out.c_str();
  fout.close();

  RCLCPP_INFO(
    node->get_logger(),
    "Saved selected passive frame transforms to: %s",
    output_filename.c_str());

  rclcpp::shutdown();
  return 0;
}
