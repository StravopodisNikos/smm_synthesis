#include <memory>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("structure_digit_setter");

  // 1) Locate package share dir (smm_synthesis)
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

  const std::string yaml_path = share_dir + "/config/yaml/assembly_6dof.yaml";

  // 2) Load assembly.yaml
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

  // 3) Generic list of structure digits that correspond to pseudojoints
  //    - metalink_1: s2, s3
  //    - metalink_2: s5, s6
  //    - metalink_3: s8, s9
  struct SDigit
  {
    std::string key;
  };

  std::vector<SDigit> sdigits = {
    {"s2"},
    {"s3"},
    {"s5"},
    {"s6"},
    {"s8"},
    {"s9"},
  };

  int count = 0;

  for (const auto & sd : sdigits) {
    if (!assembly[sd.key]) {
      RCLCPP_WARN(
        node->get_logger(),
        "[structure_digit_setter] Structure digit '%s' not found in assembly.yaml. Skipping.",
        sd.key.c_str());
      continue;
    }

    int val = assembly[sd.key].as<int>();

    // By your convention, digit == 9 → "no pseudojoint defined"
    if (val != 9) {
      ++count;
      RCLCPP_DEBUG(
        node->get_logger(),
        "[structure_digit_setter] %s = %d → counts as a pseudo-joint.",
        sd.key.c_str(), val);
    }
  }

  // 4) Basic sanity check
  // For the old 3DOF case you had 2..4 pseudos;
  // for 3–6 DOF with up to 3 metalinks you can have up to 6 pseudos.
  if (count < 2 || count > 6) {
    RCLCPP_ERROR(
      node->get_logger(),
      "[structure_digit_setter] Invalid pseudojoint count derived from assembly.yaml: %d "
      "(expected between 2 and 6).", count);
    rclcpp::shutdown();
    return 1;
  }

  // 5) Declare parameter on this node
  node->declare_parameter<int>("STRUCTURE_DIGIT", count);
  RCLCPP_INFO(
    node->get_logger(),
    "[structure_digit_setter] Declared parameter STRUCTURE_DIGIT = %d", count);

  // Keep node spinning so other nodes can query this parameter
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
