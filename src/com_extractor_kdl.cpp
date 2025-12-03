#include <chrono>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <urdf/model.h>  // or <urdf/model.hpp> if you prefer
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

// Struct to hold mass and world-frame weighted CoM
struct AggregatedMass
{
  double mass;
  Eigen::Vector3d weighted_world_com;

  AggregatedMass()
  : mass(0.0),
    weighted_world_com(Eigen::Vector3d::Zero())
  {}
};

Eigen::Vector3d KDLFrameToPosition(const KDL::Frame & frame)
{
  return Eigen::Vector3d(frame.p.x(), frame.p.y(), frame.p.z());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("com_extractor_kdl");
  auto logger = node->get_logger();

  // --- Parameters ---

  // 1) output_file: full path where we will save the YAML
  node->declare_parameter<std::string>("output_file", "");
  std::string output_file;
  node->get_parameter("output_file", output_file);

  if (output_file.empty()) {
    RCLCPP_ERROR(logger, "Parameter 'output_file' is empty.");
    rclcpp::shutdown();
    return 1;
  }

  // 2) robot_description: URDF string
  node->declare_parameter<std::string>("robot_description", "");
  std::string robot_description;
  node->get_parameter("robot_description", robot_description);

  if (robot_description.empty()) {
    RCLCPP_ERROR(
      logger,
      "Parameter 'robot_description' is empty. Make sure it is set in the launch file."
    );
    rclcpp::shutdown();
    return 1;
  }

  // small delay 
  rclcpp::sleep_for(std::chrono::seconds(2));
  RCLCPP_INFO(logger, "Starting CoM extraction...");

  // --- Build URDF model from robot_description ---
  urdf::Model model;
  if (!model.initString(robot_description)) {
    RCLCPP_ERROR(logger, "Failed to parse URDF from robot_description parameter.");
    rclcpp::shutdown();
    return 1;
  }

  // --- Build KDL tree ---
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)) {
    RCLCPP_ERROR(logger, "Failed to construct KDL tree.");
    rclcpp::shutdown();
    return 1;
  }

  std::string world_frame = kdl_tree.getRootSegment()->first;
  RCLCPP_INFO(logger, "Using world frame: %s", world_frame.c_str());

  // Output map: frame_name (mapped) -> world-frame CoM
  std::map<std::string, Eigen::Vector3d> aggregated_coms;

  AggregatedMass current_aggregation;
  std::string current_group_name;

  // Start from the first child of the root link (same logic as your ROS1 code)
  urdf::LinkConstSharedPtr current_link = model.getRoot();
  if (!current_link) {
    RCLCPP_ERROR(logger, "Null root link.");
    rclcpp::shutdown();
    return 1;
  }

  current_link = current_link->child_links.empty() ? nullptr : current_link->child_links[0];
  if (!current_link) {
    RCLCPP_ERROR(logger, "World has no child links.");
    rclcpp::shutdown();
    return 1;
  }

  current_group_name = current_link->name;

  // Mapping from link name to YAML key
  std::map<std::string, std::string> name_map = {
    {"base_link", "gsl00"},
    {"active_module_b_1", "gsl10"},
    {"active_module_b_2", "gsl20"}
  };

  while (current_link)
  {
    // Build KDL chain from world to current link
    KDL::Chain chain;
    if (!kdl_tree.getChain(world_frame, current_link->name, chain)) {
      RCLCPP_ERROR(logger, "Failed to get chain to %s", current_link->name.c_str());
      break;
    }

    // FK at zero joint positions
    KDL::Frame world_T_link = KDL::Frame::Identity();
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
      world_T_link = world_T_link * chain.getSegment(i).pose(0.0);
    }

    // Accumulate mass and weighted CoM
    if (current_link->inertial) {
      double mass = current_link->inertial->mass;

      KDL::Vector local_offset(
        current_link->inertial->origin.position.x,
        current_link->inertial->origin.position.y,
        current_link->inertial->origin.position.z
      );

      KDL::Vector world_com = world_T_link * local_offset;
      Eigen::Vector3d world_com_vec(world_com.x(), world_com.y(), world_com.z());

      current_aggregation.mass += mass;
      current_aggregation.weighted_world_com += mass * world_com_vec;
    }

    // If this is a leaf, store the aggregation (if any) and stop
    if (current_link->child_joints.empty()) {
      if (current_aggregation.mass > 0.0 && name_map.count(current_group_name)) {
        Eigen::Vector3d final_com =
          current_aggregation.weighted_world_com / current_aggregation.mass;
        aggregated_coms[name_map[current_group_name]] = final_com;
      }
      break;
    }

    // Traverse next link (assuming single chain)
    urdf::JointConstSharedPtr child_joint = current_link->child_joints[0];
    urdf::LinkConstSharedPtr next_link = current_link->child_links[0];

    if (!child_joint || !next_link) {
      RCLCPP_ERROR(logger, "Invalid child joint or link at: %s", current_link->name.c_str());
      break;
    }

    if (child_joint->type == urdf::Joint::FIXED) {
      // Same group, accumulate
      current_link = next_link;
    } else {
      // New group: store old aggregation (if valid)
      if (current_aggregation.mass > 0.0 && name_map.count(current_group_name)) {
        Eigen::Vector3d final_com =
          current_aggregation.weighted_world_com / current_aggregation.mass;
        aggregated_coms[name_map[current_group_name]] = final_com;
      }

      // Reset aggregation and move to next link
      current_aggregation = AggregatedMass();
      current_link = next_link;
      current_group_name = current_link->name;
    }
  }

  // --- Write YAML directly to output_file (no package prefixes) ---
  YAML::Emitter out;
  out << YAML::BeginMap;
  for (const auto & kv : aggregated_coms) {
    out << YAML::Key << kv.first << YAML::Value << YAML::BeginSeq;
    out << kv.second.x() << kv.second.y() << kv.second.z();
    out << YAML::EndSeq;
  }
  out << YAML::EndMap;

  std::ofstream fout(output_file);
  if (!fout.is_open()) {
    RCLCPP_ERROR(
      logger,
      "Failed to open file for writing: %s",
      output_file.c_str()
    );
    rclcpp::shutdown();
    return 1;
  }

  fout << out.c_str();
  fout.close();

  RCLCPP_INFO(logger, "World-frame CoMs saved to: %s", output_file.c_str());

  rclcpp::shutdown();
  return 0;
}
