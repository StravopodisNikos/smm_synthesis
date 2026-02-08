#include <memory>
#include <string>
#include <map>
#include <fstream>
#include <sstream>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <urdf/model.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>

#include "rclcpp/rclcpp.hpp"

#include "smm_screws/core/ScrewsMain.h"

// FOR TEST RUN (ROS 2):
//   ros2 run smm_synthesis inertia_extractor_kdl \
//     --ros-args -p output_file:=Mscomi0.yaml
// Because we never need “transform from base_link to tcp_link” in this node 
// (we’re expressing everything in the world/root frame), there’s no need 
// to pass base/tcp names.

struct AggregatedInertia {
  double mass = 0.0;
  Eigen::Vector3d weighted_com = Eigen::Vector3d::Zero();
  Eigen::Matrix3d inertia_world = Eigen::Matrix3d::Zero();
};

Eigen::Matrix3d KDLRotationToEigen(const KDL::Frame & frame)
{
  Eigen::Matrix3d mat;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      mat(i, j) = frame.M(i, j);
    }
  }
  return mat;
}

// Use ScrewsMain::skew (float) but keep double in this node
Eigen::Matrix3d SkewSymmetric(const Eigen::Vector3d & v)
{
  static ScrewsMain screws;  // one instance, reused

  // Convert Eigen::Vector3d -> Eigen::Vector3f
  Eigen::Vector3f vf = v.cast<float>();

  // Call your core implementation
  Eigen::Matrix3f mf = screws.skew(vf);

  // Convert back to double to match the rest of this node
  return mf.cast<double>();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("inertia_extractor_kdl_ndof");

  using namespace std::chrono_literals;

  // ---------------------------------------------------------------------------
  // 1) Parameters: output_file (full path) + robot_description (URDF string)
  // ---------------------------------------------------------------------------
  node->declare_parameter<std::string>("output_file", "");
  std::string output_filename;
  node->get_parameter("output_file", output_filename);

  if (output_filename.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "No 'output_file' parameter given. Use --ros-args -p output_file:=Mscomi0.yaml");
    return 1;
  }

  // small delay if launched together with other nodes
  rclcpp::sleep_for(2s);

  node->declare_parameter<std::string>("robot_description", "");
  std::string robot_description;
  node->get_parameter("robot_description", robot_description);

  if (robot_description.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Parameter 'robot_description' is empty. Set it in your launch file.");
    return 1;
  }

  // ---------------------------------------------------------------------------
  // 2) Parse URDF and build KDL tree
  // ---------------------------------------------------------------------------
  urdf::Model model;
  if (!model.initString(robot_description)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to parse URDF from 'robot_description'.");
    return 1;
  }

  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to construct KDL tree from URDF.");
    return 1;
  }

  urdf::LinkConstSharedPtr current_link = model.getRoot();
  if (!current_link) {
    RCLCPP_ERROR(node->get_logger(), "Root link is NULL.");
    return 1;
  }

  const std::string root_name = kdl_tree.getRootSegment()->first;
  RCLCPP_INFO(node->get_logger(), "KDL root link: '%s'", root_name.c_str());
  RCLCPP_INFO(node->get_logger(),
              "Traversing single chain root->leaf, aggregating fixed segments.");

  // ---------------------------------------------------------------------------
  // 3) Aggregation data
  // ---------------------------------------------------------------------------
  AggregatedInertia current_agg;
  int group_index = 0;        // group 0 = environment (world/base), then 1,2,... for bodies
  bool first_group = true;

  // Output: Mscom00, Mscom10, Mscom20, ..., one per DOF (3..6)
  std::map<std::string, Eigen::Matrix<double, 6, 6>> spatial_inertias;

  auto finalize_group = [&](const std::string & group_root_link)
  {
    if (current_agg.mass <= 0.0) {
      // No mass in this group – just reset, but do NOT advance group_index
      RCLCPP_WARN(
        node->get_logger(),
        "Group rooted at '%s' has zero mass, skipping.",
        group_root_link.c_str());
      current_agg = AggregatedInertia{};
      return;
    }

    Eigen::Vector3d com = current_agg.weighted_com / current_agg.mass;
    Eigen::Matrix3d skew = SkewSymmetric(com);

    Eigen::Matrix<double, 6, 6> M;
    M.setZero();
    // NOTE: this matches your existing 3-DOF extractor pattern
    M.block<3,3>(0,0) = current_agg.mass * Eigen::Matrix3d::Identity();
    M.block<3,3>(0,3) = -skew;
    M.block<3,3>(3,0) =  skew;
    M.block<3,3>(3,3) = current_agg.inertia_world;

    if (first_group) {
      // Treat the first aggregated group as "environment" (world + table base, etc.)
      RCLCPP_INFO(
        node->get_logger(),
        "Skipping environment group rooted at '%s' (mass = %.4f)",
        group_root_link.c_str(), current_agg.mass);
      first_group = false;
    } else {
      // Real robot body group => assign index: Mscom00, Mscom10, Mscom20, ...
      int body_idx = group_index - 1;  // group_index 1 -> body 0, etc.

      std::ostringstream key_stream;
      key_stream << "Mscom" << body_idx << "0";
      const std::string key = key_stream.str();

      spatial_inertias[key] = M;

      RCLCPP_INFO(
        node->get_logger(),
        "Stored spatial inertia for body %d (root='%s') as '%s'",
        body_idx, group_root_link.c_str(), key.c_str());
    }

    current_agg = AggregatedInertia{};
    ++group_index;
  };

  // ---------------------------------------------------------------------------
  // 4) Walk the chain root -> leaf, splitting at non-fixed joints
  // ---------------------------------------------------------------------------
  std::string current_group_root = current_link->name;

  while (current_link) {
    // Build KDL chain from root to this link
    KDL::Chain chain;
    if (!kdl_tree.getChain(root_name, current_link->name, chain)) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Chain extraction failed from '%s' to '%s'",
        root_name.c_str(), current_link->name.c_str());
      break;
    }

    // Compute world_T_link
    KDL::Frame world_T = KDL::Frame::Identity();
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
      world_T = world_T * chain.getSegment(i).pose(0.0);
    }

    Eigen::Matrix3d R_world = KDLRotationToEigen(world_T);
    Eigen::Vector3d p_world(world_T.p.x(), world_T.p.y(), world_T.p.z());

    // Accumulate this link's inertia (if any) into the current group
    if (current_link->inertial) {
      double m = current_link->inertial->mass;

      Eigen::Vector3d c_local(
        current_link->inertial->origin.position.x,
        current_link->inertial->origin.position.y,
        current_link->inertial->origin.position.z);

      Eigen::Matrix3d I_local;
      I_local << current_link->inertial->ixx, current_link->inertial->ixy, current_link->inertial->ixz,
                 current_link->inertial->ixy, current_link->inertial->iyy, current_link->inertial->iyz,
                 current_link->inertial->ixz, current_link->inertial->iyz, current_link->inertial->izz;

      // CoM in world frame
      Eigen::Vector3d c_world = p_world + R_world * c_local;

      // Inertia rotated to world frame (still about local CoM)
      Eigen::Matrix3d I_rotated = R_world * I_local * R_world.transpose();

      // Parallel axis: shift from CoM to world origin
      Eigen::Matrix3d I_shifted =
        I_rotated + m * ((c_world.dot(c_world)) * Eigen::Matrix3d::Identity() -
                         c_world * c_world.transpose());

      current_agg.mass          += m;
      current_agg.weighted_com  += m * c_world;
      current_agg.inertia_world += I_shifted;
    }

    // If leaf, finalize and stop
    if (current_link->child_joints.empty()) {
      finalize_group(current_group_root);
      break;
    }

    // Otherwise, follow **single** child joint (serial chain assumption)
    auto child_joint = current_link->child_joints[0];
    auto next_link   = current_link->child_links[0];

    if (!child_joint || !next_link) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Invalid child joint or link at: %s",
        current_link->name.c_str());
      break;
    }

    if (child_joint->type == urdf::Joint::FIXED) {
      // Still same rigid "body"
      current_link = next_link;
    } else {
      // Non-fixed joint => end of this body, start a new one
      finalize_group(current_group_root);
      current_link       = next_link;
      current_group_root = current_link->name;
    }
  }

  // ---------------------------------------------------------------------------
  // 5) Write YAML (full path output_filename)
  // ---------------------------------------------------------------------------
  int nbodies = static_cast<int>(spatial_inertias.size());

  if (nbodies < 3 || nbodies > 6) {
    RCLCPP_WARN(
      node->get_logger(),
      "Extracted %d spatial inertia bodies (Mscomxx). "
      "Expected between 3 and 6 for SMM Ndof.",
      nbodies);
  } else {
    RCLCPP_INFO(
      node->get_logger(),
      "Extracted %d-DoF spatial inertias (3–6 DOF range).",
      nbodies);
  }

  YAML::Emitter out;
  out << YAML::BeginMap;
  for (const auto & item : spatial_inertias) {
    out << YAML::Key << item.first << YAML::Value << YAML::BeginSeq;
    const auto & M = item.second;
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) {
        out << M(i, j);
      }
    }
    out << YAML::EndSeq;
  }
  out << YAML::EndMap;


  std::ofstream fout(output_filename);
  if (!fout.is_open()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to open output file: %s", output_filename.c_str());
    return 1;
  }
  fout << out.c_str();
  fout.close();

  RCLCPP_INFO(
    node->get_logger(),
    "Spatial inertias saved to: %s",
    output_filename.c_str());

  rclcpp::shutdown();
  return 0;
}
