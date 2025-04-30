#include <ros/ros.h>
#include <ros/package.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <map>
#include <vector>

// Struct to hold mass and world-frame weighted CoM
struct AggregatedMass
{
    double mass;
    Eigen::Vector3d weighted_world_com;

    AggregatedMass() : mass(0.0), weighted_world_com(Eigen::Vector3d::Zero()) {}
};

Eigen::Vector3d KDLFrameToPosition(const KDL::Frame& frame)
{
    return Eigen::Vector3d(frame.p.x(), frame.p.y(), frame.p.z());
}

Eigen::Matrix3d KDLRotationToMatrix(const KDL::Frame& frame)
{
    Eigen::Matrix3d mat;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            mat(i,j) = frame.M(i,j);
        }
    }
    return mat;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "com_extractor_kdl");
    ros::NodeHandle nh("~");  // Private NodeHandle

    // 1. Get output YAML file name from parameter
    std::string output_filename;
    if (!nh.getParam("output_file", output_filename))
    {
        ROS_ERROR("No output_file parameter given. Please specify where to save the YAML!");
        return -1;
    }

    // 2. Delay to allow robot to load
    ros::Duration(2.0).sleep();
    ROS_INFO("Starting aggregated world CoM extraction...");

    // 3. Load URDF model
    urdf::Model model;
    if (!model.initParam("robot_description"))
    {
        ROS_ERROR("Failed to parse URDF from parameter server");
        return -1;
    }

    // 4. Load KDL Tree from URDF
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
    {
        ROS_ERROR("Failed to construct KDL tree");
        return -1;
    }

    ROS_INFO("Successfully loaded KDL Tree!");

    // 5. Start from root link
    urdf::LinkConstSharedPtr current_link = model.getRoot();
    std::string root_name = kdl_tree.getRootSegment()->first;

    if (!current_link)
    {
        ROS_ERROR("Root link is NULL. Cannot proceed.");
        return -1;
    }

    AggregatedMass current_aggregation;
    std::string current_group_name = current_link->name;

    // Map to hold final aggregated CoMs
    std::map<std::string, Eigen::Vector3d> aggregated_coms;

    // 6. Traverse the URDF tree
    while (current_link)
    {
        // 6.1 Compute world transform of the current link
        KDL::Chain chain;
        if (!kdl_tree.getChain(root_name, current_link->name, chain))
        {
            ROS_ERROR_STREAM("Failed to extract KDL chain to link: " << current_link->name);
            break;
        }

        KDL::Frame world_T_link = KDL::Frame::Identity();
        for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i)
        {
            world_T_link = world_T_link * chain.getSegment(i).pose(0.0);  // Zero joint angles
        }

        // 6.2 If link has inertial, compute its CoM in world frame
        if (current_link->inertial)
        {
            double mass = current_link->inertial->mass;
            KDL::Vector local_com_offset(
                current_link->inertial->origin.position.x,
                current_link->inertial->origin.position.y,
                current_link->inertial->origin.position.z
            );

            // Full CoM in world = world_T_link * local CoM offset
            KDL::Vector world_com = world_T_link * local_com_offset;
            Eigen::Vector3d world_com_eigen(world_com.x(), world_com.y(), world_com.z());

            current_aggregation.mass += mass;
            current_aggregation.weighted_world_com += mass * world_com_eigen;
        }

        // 6.3 If no children, finalize
        if (current_link->child_joints.empty())
        {
            if (current_aggregation.mass > 0.0)
            {
                Eigen::Vector3d final_com = current_aggregation.weighted_world_com / current_aggregation.mass;
                aggregated_coms[current_group_name] = final_com;
            }
            break;
        }

        // 6.4 Continue based on joint type
        urdf::JointConstSharedPtr child_joint = current_link->child_joints[0];
        urdf::LinkConstSharedPtr next_link = current_link->child_links[0];

        if (!child_joint || !next_link)
        {
            ROS_ERROR_STREAM("Invalid child joint or link for: " << current_link->name);
            break;
        }

        if (child_joint->type == urdf::Joint::FIXED)
        {
            // Fixed joint → keep aggregating
            current_link = next_link;
        }
        else
        {
            // Moving joint (revolute or prismatic) → finalize aggregation
            if (current_aggregation.mass > 0.0)
            {
                Eigen::Vector3d final_com = current_aggregation.weighted_world_com / current_aggregation.mass;
                aggregated_coms[current_group_name] = final_com;
            }

            // Reset aggregation
            current_aggregation = AggregatedMass();
            current_link = next_link;
            current_group_name = current_link->name;
        }
    }

    ROS_INFO("Aggregated world CoM computed for %lu physical links.", aggregated_coms.size());

    // 7. Print result
    for (const auto& entry : aggregated_coms)
    {
        std::cout << "\nAggregated CoM (world frame) for link group: " << entry.first << "\n";
        std::cout << "[" << entry.second.x() << ", " << entry.second.y() << ", " << entry.second.z() << "]\n";
    }

    // 8. Save to YAML
    std::string save_path = ros::package::getPath("smm_synthesis") + "/config/yaml/" + output_filename;

    YAML::Emitter out;
    out << YAML::BeginMap;
    for (const auto& entry : aggregated_coms)
    {
        out << YAML::Key << entry.first << YAML::Value << YAML::BeginSeq;
        out << entry.second.x() << entry.second.y() << entry.second.z();
        out << YAML::EndSeq;
    }
    out << YAML::EndMap;

    std::ofstream fout(save_path);
    fout << out.c_str();
    fout.close();

    ROS_INFO_STREAM("Aggregated world CoMs saved to: " << save_path);

    ros::spin();
    return 0;
}
