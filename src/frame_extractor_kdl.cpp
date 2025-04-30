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

    // Rotation part
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            mat(i,j) = frame.M(i,j);
        }
    }

    // Translation part
    mat(0,3) = frame.p.x();
    mat(1,3) = frame.p.y();
    mat(2,3) = frame.p.z();

    return mat;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frame_extractor_kdl");
    ros::NodeHandle nh("~");  // Private NodeHandle for private parameters

    // 1. Get output YAML file name from parameter
    std::string output_filename;
    if (!nh.getParam("output_file", output_filename))
    {
        ROS_ERROR("No output_file parameter given. Please specify where to save the YAML!");
        return -1;
    }

    // 2. Optional: wait a bit for robot_state_publisher to fully load
    ros::Duration(2.0).sleep();
    ROS_INFO("Starting frame extraction...");

    // 3. Load URDF from parameter server
    urdf::Model model;
    if (!model.initParam("robot_description"))
    {
        ROS_ERROR("Failed to parse URDF from parameter server");
        return -1;
    }

    // 4. Create KDL tree
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
    {
        ROS_ERROR("Failed to construct KDL tree");
        return -1;
    }
    ROS_INFO("Successfully loaded KDL Tree!");

    // 5. Extract root name (could be "world" or "base_link")
    std::string root_name = kdl_tree.getRootSegment()->first;
    ROS_INFO_STREAM("Root link detected: " << root_name);

    // 6. Find active joints only and compute their world frames
    std::map<std::string, Eigen::Matrix4d> frame_map;

    for (const auto& segment : kdl_tree.getSegments())
    {
        const auto& seg = segment.second.segment;
        if (seg.getJoint().getType() != KDL::Joint::None)  // Only active joints
        {
            std::string joint_name = seg.getName();

            // Create a KDL chain from root to this joint
            KDL::Chain chain;
            if (!kdl_tree.getChain(root_name, joint_name, chain))
            {
                ROS_ERROR_STREAM("Failed to extract chain from " << root_name << " to " << joint_name);
                continue;
            }

            // Compute forward kinematics along the chain
            KDL::Frame frame = KDL::Frame::Identity();
            for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i)
            {
                const KDL::Segment& chain_seg = chain.getSegment(i);
                frame = frame * chain_seg.pose(0.0);  // 0 angle at home configuration
            }

            Eigen::Matrix4d mat = KDLFrameToEigen(frame);
            frame_map[joint_name] = mat;
        }
    }

    ROS_INFO("Found %lu active joints.", frame_map.size());

    // 7. Print matrices
    for (const auto& entry : frame_map)
    {
        std::cout << "\nFrame: " << entry.first << "\n";
        std::cout << entry.second << "\n";
    }

    // 8. Build save path inside your package
    std::string save_path = ros::package::getPath("smm_synthesis") + "/config/yaml/" + output_filename;

    // 9. Save matrices into YAML
    YAML::Emitter out;
    out << YAML::BeginMap;
    for (const auto& entry : frame_map)
    {
        out << YAML::Key << entry.first << YAML::Value << YAML::BeginSeq;
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                out << entry.second(i,j);
            }
        }
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
