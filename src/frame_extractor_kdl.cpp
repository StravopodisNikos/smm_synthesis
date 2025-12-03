#include <chrono>
#include <fstream>
#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <urdf/model.hpp>  // or <urdf/model.hpp> in newer headers
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

Eigen::Matrix4d KDLFrameToEigen(const KDL::Frame& frame)
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

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("frame_extractor_kdl");

    // --- Parameters ---
    // 1) Declare & get output_file as FULL PATH
    node->declare_parameter<std::string>("output_file", "");
    std::string output_file;
    node->get_parameter("output_file", output_file);

    if (output_file.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Parameter 'output_file' is empty.");
        return 1;
    }

    // 2) robot_description already passed from launch
    node->declare_parameter<std::string>("robot_description", "");
    std::string robot_description;
    node->get_parameter("robot_description", robot_description);
    if (robot_description.empty()) {
        RCLCPP_ERROR(
            node->get_logger(),
            "Parameter 'robot_description' is empty. Make sure it is set in the launch file."
        );
        return 1;
    }

    // Small delay 
    rclcpp::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(node->get_logger(), "Starting frame extraction...");

    // 3) Build URDF model from robot_description param
    urdf::Model model;
    if (!model.initString(robot_description)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to parse URDF from robot_description parameter.");
        return 1;
    }

    // 4) Build KDL tree
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to construct KDL tree.");
        rclcpp::shutdown();
        return 1;
    }

    std::string root_name = "base_link";
    RCLCPP_INFO(node->get_logger(),
                "Setting root frame to: '%s' with rotz(-pi) compensation",
                root_name.c_str());

    // Map of URDF link → internal name
    std::map<std::string, std::string> frame_name_map = {
        {"base_link",         "gsa00"},
        {"active_module_b_1", "gsa10"},
        {"active_module_b_2", "gsa20"}
    };

    std::map<std::string, Eigen::Matrix4d> extracted_frames;

    // Define rotz(-pi) = diag(-1, -1, 1, 1)
    Eigen::Matrix4d Rz_pi = Eigen::Matrix4d::Identity();
    Rz_pi(0, 0) = -1.0;
    Rz_pi(1, 1) = -1.0;

    // --- Extract transforms for each frame ---
    for (const auto& pair : frame_name_map)
    {
        const std::string& urdf_frame    = pair.first;
        const std::string& internal_name = pair.second;

        KDL::Chain chain;
        if (!kdl_tree.getChain(root_name, urdf_frame, chain)) {
            RCLCPP_ERROR(node->get_logger(),
                         "Failed to extract chain from '%s' to '%s'",
                         root_name.c_str(), urdf_frame.c_str());
            continue;
        }

        KDL::Frame frame = KDL::Frame::Identity();
        for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
            const KDL::Segment& segment = chain.getSegment(i);
            frame = frame * segment.pose(0.0);
        }

        Eigen::Matrix4d tf = KDLFrameToEigen(frame);
        Eigen::Matrix4d transformed = Rz_pi * tf;

        extracted_frames[internal_name] = transformed;

        RCLCPP_INFO(node->get_logger(),
                    "Mapped URDF frame '%s' → '%s' with rotz(-pi)",
                    urdf_frame.c_str(), internal_name.c_str());
    }

    // --- Save directly to output_file (already full path) ---
    YAML::Emitter out;
    out << YAML::BeginMap;
    for (const auto& item : extracted_frames)
    {
        out << YAML::Key << item.first << YAML::Value << YAML::BeginSeq;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                out << item.second(i, j);
        out << YAML::EndSeq;
    }
    out << YAML::EndMap;

    std::ofstream fout(output_file);
    if (!fout.is_open()) {
        RCLCPP_ERROR(
            node->get_logger(),
            "Failed to open output file: '%s'", output_file.c_str()
        );
        return 1;
    }
    fout << out.c_str();
    fout.close();

    RCLCPP_INFO(
        node->get_logger(),
        "Frames saved to: %s",
        output_file.c_str()
    );

    rclcpp::shutdown();
    return 0;
}
