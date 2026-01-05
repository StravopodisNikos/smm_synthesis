#include <chrono>
#include <fstream>
#include <string>
#include <map>

#include <rclcpp/rclcpp.hpp>

#include <urdf/model.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

static Eigen::Matrix4d KDLFrameToEigen(const KDL::Frame& frame)
{
  Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      mat(i, j) = frame.M(i, j);

  mat(0, 3) = frame.p.x();
  mat(1, 3) = frame.p.y();
  mat(2, 3) = frame.p.z();
  return mat;
}

static Eigen::Matrix4d RotZMinusPi()
{
  Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
  R(0, 0) = -1.0;
  R(1, 1) = -1.0;
  return R;
}

static bool isRotationalJoint(const KDL::Joint& j)
{
  // Treat only rotational joints as "active"
  const auto t = j.getType();
  return (t == KDL::Joint::RotAxis ||
          t == KDL::Joint::RotX ||
          t == KDL::Joint::RotY ||
          t == KDL::Joint::RotZ);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("active_frames_extractor_ndof");

  // Parameters
  node->declare_parameter<std::string>("output_file", "");
  node->declare_parameter<std::string>("robot_description", "");
  node->declare_parameter<std::string>("root_link", "base_link");
  node->declare_parameter<std::string>("tip_link", "tcp");
  node->declare_parameter<bool>("apply_rotz_minus_pi", true);

  std::string output_file, robot_description, root_link, tip_link;
  bool apply_rotz_minus_pi = true;

  node->get_parameter("output_file", output_file);
  node->get_parameter("robot_description", robot_description);
  node->get_parameter("root_link", root_link);
  node->get_parameter("tip_link", tip_link);
  node->get_parameter("apply_rotz_minus_pi", apply_rotz_minus_pi);

  if (output_file.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'output_file' is empty.");
    rclcpp::shutdown();
    return 1;
  }
  if (robot_description.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'robot_description' is empty.");
    rclcpp::shutdown();
    return 1;
  }

  // Optional delay (helps if your system sets params slightly later)
  rclcpp::sleep_for(std::chrono::seconds(1));
  RCLCPP_INFO(node->get_logger(), "Extracting active frames from '%s' -> '%s' ...",
              root_link.c_str(), tip_link.c_str());

  // Parse URDF
  urdf::Model model;
  if (!model.initString(robot_description)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to parse URDF from robot_description.");
    rclcpp::shutdown();
    return 1;
  }

  // Build KDL tree
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to construct KDL tree.");
    rclcpp::shutdown();
    return 1;
  }

  // Build KDL chain root->tip
  KDL::Chain chain;
  if (!kdl_tree.getChain(root_link, tip_link, chain)) {
    RCLCPP_ERROR(node->get_logger(),
                 "Failed to extract chain from '%s' to '%s'. Check link names.",
                 root_link.c_str(), tip_link.c_str());
    rclcpp::shutdown();
    return 1;
  }

  // Compensation
  Eigen::Matrix4d C = Eigen::Matrix4d::Identity();
  if (apply_rotz_minus_pi) {
    C = RotZMinusPi();
    RCLCPP_INFO(node->get_logger(), "Applying rotz(-pi) compensation to all frames.");
  } else {
    RCLCPP_INFO(node->get_logger(), "No global compensation applied.");
  }

  // Extract frames incrementally along the chain
  std::map<std::string, Eigen::Matrix4d> frames_out;

  // gsa00 = compensated identity (root frame)
  frames_out["gsa00"] = C * Eigen::Matrix4d::Identity();

  KDL::Frame T = KDL::Frame::Identity();
  int active_idx = 0;

  for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
    const KDL::Segment& seg = chain.getSegment(i);

    // q=0 for pose extraction
    T = T * seg.pose(0.0);

    const KDL::Joint& j = seg.getJoint();
    if (!isRotationalJoint(j)) {
      continue;
    }

    // This segment introduces an active rotational joint.
    ++active_idx;

    Eigen::Matrix4d tf = KDLFrameToEigen(T);
    Eigen::Matrix4d tf_comp = C * tf;

    // Naming: gsa10, gsa20, ..., gsaN0
    const std::string key = "gsa" + std::to_string(active_idx) + "0";
    frames_out[key] = tf_comp;

    RCLCPP_INFO(node->get_logger(),
                "Active #%d: segment='%s', joint='%s' -> key='%s'",
                active_idx,
                seg.getName().c_str(),
                j.getName().c_str(),
                key.c_str());
  }

  if (active_idx < 3 || active_idx > 6) {
    RCLCPP_WARN(node->get_logger(),
                "Detected %d rotational joints on chain. Expected 3..6. "
                "Continuing anyway.", active_idx);
  } else {
    RCLCPP_INFO(node->get_logger(), "Detected %d-DOF active chain.", active_idx);
  }

  // Write YAML (16 values per frame, row-major)
  YAML::Emitter out;
  out << YAML::BeginMap;
  for (const auto& item : frames_out) {
    out << YAML::Key << item.first << YAML::Value << YAML::BeginSeq;
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        out << item.second(r, c);
    out << YAML::EndSeq;
  }
  out << YAML::EndMap;

  std::ofstream fout(output_file);
  if (!fout.is_open()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to open output file: '%s'", output_file.c_str());
    rclcpp::shutdown();
    return 1;
  }
  fout << out.c_str();
  fout.close();

  RCLCPP_INFO(node->get_logger(), "Saved active frames YAML to: %s", output_file.c_str());

  rclcpp::shutdown();
  return 0;
}
