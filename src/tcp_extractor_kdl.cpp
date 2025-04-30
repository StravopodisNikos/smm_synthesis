#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tcp_extractor_kdl");
    ros::NodeHandle nh("~");  // use private handle for params

    // Get output file name
    std::string output_filename;
    if (!nh.getParam("output_file", output_filename))
    {
        ROS_ERROR("No output_file parameter given.");
        return -1;
    }

    // Load URDF and parse KDL tree
    urdf::Model model;
    if (!model.initParam("robot_description"))
    {
        ROS_ERROR("Failed to load robot_description.");
        return -1;
    }

    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
    {
        ROS_ERROR("Failed to parse KDL tree from URDF.");
        return -1;
    }

    const std::string base_link = kdl_tree.getRootSegment()->first;
    const std::string tip_link = "massage_tool";
    KDL::Chain chain;
    if (!kdl_tree.getChain(base_link, tip_link, chain))
    {
        ROS_ERROR("Failed to extract chain from %s to %s", base_link.c_str(), tip_link.c_str());
        return -1;
    }

    // Add fixed frame from massage_tool to tcp
    KDL::Frame massage_tool_to_tcp_frame(
        KDL::Rotation::Quaternion(0, 0, 0, 1),
        KDL::Vector(0.00, 0.00, 0.065)
    );
    KDL::Segment fixed_tcp_segment("tcp", KDL::Joint(KDL::Joint::None), massage_tool_to_tcp_frame);
    chain.addSegment(fixed_tcp_segment);

    // Compute FK with all joints at zero
    KDL::Frame world_to_tcp = KDL::Frame::Identity();
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i)
    {
        const KDL::Segment& segment = chain.getSegment(i);
        world_to_tcp = world_to_tcp * segment.pose(0.0);
    }

    // Convert to Eigen 4x4
    Eigen::Matrix4d tf_matrix = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 3; ++i)
    {
        tf_matrix(i, 3) = world_to_tcp.p[i];
        for (int j = 0; j < 3; ++j)
            tf_matrix(i, j) = world_to_tcp.M(i, j);
    }

    std::cout << "\nTransform from world to TCP (4x4 matrix):\n" << tf_matrix << "\n";

    // Save to YAML
    std::string save_path = ros::package::getPath("smm_synthesis") + "/config/yaml/" + output_filename;

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "tcp_frame" << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            out << tf_matrix(i, j);
    out << YAML::EndSeq;
    out << YAML::EndMap;

    std::ofstream fout(save_path);
    fout << out.c_str();
    fout.close();

    ROS_INFO_STREAM("TCP frame saved to: " << save_path);

    ros::spinOnce();
    return 0;
}
