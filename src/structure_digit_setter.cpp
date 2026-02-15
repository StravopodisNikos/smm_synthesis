#include <memory>
#include <string>

#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("structure_digit_setter");

  // Locate package share dir (smm_synthesis)
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

  const std::string yaml_path = share_dir + "/config/yaml/3dof/assembly_3dof.yaml";

  // Load assembly.yaml
  YAML::Node assembly;
  try {
    assembly = YAML::LoadFile(yaml_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to load '%s': %s", yaml_path.c_str(), e.what());
    rclcpp::shutdown();
    return 1;
  }

  // Read structure digits
  int s2 = assembly["s2"].as<int>();
  int s3 = assembly["s3"].as<int>();
  int s5 = assembly["s5"].as<int>();
  int s6 = assembly["s6"].as<int>();

  int count = 0;
  if (s2 != 9) count++;
  if (s3 != 9) count++;
  if (s5 != 9) count++;
  if (s6 != 9) count++;

  if (count < 2 || count > 4) {
    RCLCPP_ERROR(
      node->get_logger(),
      "[structure_digit_setter] Invalid structure count: %d", count);
    rclcpp::shutdown();
    return 1;
  }

  // In ROS2 there is no global param server; this node defines its own parameter
  node->declare_parameter<int>("STRUCTURE_DIGIT", count);
  RCLCPP_INFO(
    node->get_logger(),
    "[structure_digit_setter] Declared parameter STRUCTURE_DIGIT = %d", count);

  // Keep node alive so other nodes can query this parameter via rclcpp::SyncParametersClient, etc.
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
