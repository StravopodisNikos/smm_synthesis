#include <ros/ros.h>
#include <ros/package.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <vector>
#include <map>

Eigen::Matrix4d KDLFrameToEigen(const KDL::Frame& frame)
{
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            mat(i,j) = frame.M(i,j);
    mat(0,3) = frame.p.x();
    mat(1,3) = frame.p.y();
    mat(2,3) = frame.p.z();
    return mat;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frame_extractor_kdl");
    ros::NodeHandle nh("~");

    std::string output_filename;
    if (!nh.getParam("output_file", output_filename)) {
        ROS_ERROR("No output_file parameter given. Please specify where to save the YAML!");
        return -1;
    }

    ros::Duration(2.0).sleep();
    ROS_INFO("Starting frame extraction...");

    urdf::Model model;
    if (!model.initParam("robot_description")) {
        ROS_ERROR("Failed to parse URDF from parameter server");
        return -1;
    }

    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)) {
        ROS_ERROR("Failed to construct KDL tree");
        return -1;
    }

    std::string root_name = "base_link";
    ROS_INFO_STREAM("Setting root frame to: " << root_name << " with rotz(-pi) compensation");

    // Map of URDF link → internal name
    std::map<std::string, std::string> frame_name_map = {
        {"base_link", "gsa00"},
        {"active_module_b_1", "gsa10"},
        {"active_module_b_2", "gsa20"}
    };

    std::map<std::string, Eigen::Matrix4d> extracted_frames;

    // Define rotz(-pi)
    Eigen::Matrix4d Rz_pi = Eigen::Matrix4d::Identity();
    Rz_pi(0,0) = -1.0;
    Rz_pi(1,1) = -1.0;

    for (const auto& pair : frame_name_map)
    {
        const std::string& urdf_frame = pair.first;
        const std::string& internal_name = pair.second;

        KDL::Chain chain;
        if (!kdl_tree.getChain(root_name, urdf_frame, chain)) {
            ROS_ERROR_STREAM("Failed to extract chain from " << root_name << " to " << urdf_frame);
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

        ROS_INFO_STREAM("Mapped URDF frame '" << urdf_frame << "' to '" << internal_name << "' with rotz(-pi)");
    }

    // Save to YAML
    std::string save_path = ros::package::getPath("smm_synthesis") + "/config/yaml/" + output_filename;

    YAML::Emitter out;
    out << YAML::BeginMap;
    for (const auto& item : extracted_frames)
    {
        out << YAML::Key << item.first << YAML::Value << YAML::BeginSeq;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                out << item.second(i,j);
        out << YAML::EndSeq;
    }
    out << YAML::EndMap;

    std::ofstream fout(save_path);
    fout << out.c_str();
    fout.close();

    ROS_INFO_STREAM("Frames saved to: " << save_path);
    ros::spin();
    return 0;
}
