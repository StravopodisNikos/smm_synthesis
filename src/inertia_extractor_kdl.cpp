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

// Struct to hold mass and inertia aggregation
struct AggregatedInertia
{
    double mass;
    Eigen::Vector3d weighted_com;
    Eigen::Matrix3d inertia_world;

    AggregatedInertia() : mass(0.0), weighted_com(Eigen::Vector3d::Zero()), inertia_world(Eigen::Matrix3d::Zero()) {}
};

// Helper: Convert KDL::Frame to Eigen::Matrix3d rotation
Eigen::Matrix3d KDLRotationToEigen(const KDL::Frame& frame)
{
    Eigen::Matrix3d mat;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            mat(i,j) = frame.M(i,j);
    return mat;
}

// Helper: Build skew-symmetric matrix
Eigen::Matrix3d SkewSymmetric(const Eigen::Vector3d& v)
{
    Eigen::Matrix3d skew;
    skew <<    0, -v(2),  v(1),
             v(2),    0, -v(0),
            -v(1),  v(0),    0;
    return skew;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "inertia_extractor_kdl");
    ros::NodeHandle nh("~");  // Private node handle

    // 1. Get output YAML file name
    std::string output_filename;
    if (!nh.getParam("output_file", output_filename))
    {
        ROS_ERROR("No output_file parameter given. Please specify where to save the YAML!");
        return -1;
    }

    // 2. Wait for robot to load
    ros::Duration(2.0).sleep();
    ROS_INFO("Starting aggregated world inertia extraction...");

    // 3. Load URDF
    urdf::Model model;
    if (!model.initParam("robot_description"))
    {
        ROS_ERROR("Failed to parse URDF from parameter server");
        return -1;
    }

    // 4. Parse KDL Tree
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
    {
        ROS_ERROR("Failed to construct KDL tree");
        return -1;
    }
    ROS_INFO("Successfully loaded KDL Tree!");

    // 5. Start from root
    urdf::LinkConstSharedPtr current_link = model.getRoot();
    std::string root_name = kdl_tree.getRootSegment()->first;

    if (!current_link)
    {
        ROS_ERROR("Root link is NULL. Cannot proceed.");
        return -1;
    }

    AggregatedInertia current_aggregation;
    std::string current_group_name = current_link->name;

    // Map to store final results
    struct LinkInertiaData
    {
        Eigen::Matrix3d mass_matrix;
        Eigen::Matrix3d cross_mass_matrix;
        Eigen::Matrix3d inertia_matrix;
    };
    std::map<std::string, LinkInertiaData> link_inertia_map;

    // 6. Traverse URDF
    while (current_link)
    {
        // 6.1 Compute world transform for this link
        KDL::Chain chain;
        if (!kdl_tree.getChain(root_name, current_link->name, chain))
        {
            ROS_ERROR_STREAM("Failed to extract KDL chain to link: " << current_link->name);
            break;
        }

        KDL::Frame world_T_link = KDL::Frame::Identity();
        for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i)
            world_T_link = world_T_link * chain.getSegment(i).pose(0.0);

        Eigen::Matrix3d world_rotation = KDLRotationToEigen(world_T_link);
        Eigen::Vector3d world_position(world_T_link.p.x(), world_T_link.p.y(), world_T_link.p.z());

        // 6.2 If link has inertial data, process it
        if (current_link->inertial)
        {
            double mass = current_link->inertial->mass;
            Eigen::Vector3d com_local(
                current_link->inertial->origin.position.x,
                current_link->inertial->origin.position.y,
                current_link->inertial->origin.position.z
            );

            // Local inertia tensor
            Eigen::Matrix3d inertia_local;
            inertia_local << current_link->inertial->ixx, current_link->inertial->ixy, current_link->inertial->ixz,
                             current_link->inertial->ixy, current_link->inertial->iyy, current_link->inertial->iyz,
                             current_link->inertial->ixz, current_link->inertial->iyz, current_link->inertial->izz;

            // CoM in world frame
            Eigen::Vector3d com_world = world_position + world_rotation * com_local;

            // Transform inertia to world frame
            Eigen::Matrix3d inertia_world_rotated = world_rotation * inertia_local * world_rotation.transpose();

            // Apply parallel axis theorem
            Eigen::Matrix3d parallel_axis = mass * ((com_world.dot(com_world)) * Eigen::Matrix3d::Identity() - com_world * com_world.transpose());
            Eigen::Matrix3d shifted_inertia = inertia_world_rotated + parallel_axis;

            // Aggregate
            current_aggregation.mass += mass;
            current_aggregation.weighted_com += mass * com_world;
            current_aggregation.inertia_world += shifted_inertia;
        }

        // 6.3 Traverse child joint
        if (current_link->child_joints.empty())
        {
            // No more children, finalize current aggregation
            if (current_aggregation.mass > 0.0)
            {
                Eigen::Vector3d final_com = current_aggregation.weighted_com / current_aggregation.mass;
                Eigen::Matrix3d skew_com = SkewSymmetric(final_com);

                LinkInertiaData data;
                data.mass_matrix = current_aggregation.mass * Eigen::Matrix3d::Identity();
                data.cross_mass_matrix = current_aggregation.mass * skew_com;
                data.inertia_matrix = current_aggregation.inertia_world;

                link_inertia_map[current_group_name] = data;
            }
            break;
        }

        urdf::JointConstSharedPtr child_joint = current_link->child_joints[0];
        urdf::LinkConstSharedPtr next_link = current_link->child_links[0];

        if (!child_joint || !next_link)
        {
            ROS_ERROR_STREAM("Invalid child joint or link for: " << current_link->name);
            break;
        }

        if (child_joint->type == urdf::Joint::FIXED)
        {
            current_link = next_link;  // Keep aggregating
        }
        else
        {
            // Finalize current aggregation
            if (current_aggregation.mass > 0.0)
            {
                Eigen::Vector3d final_com = current_aggregation.weighted_com / current_aggregation.mass;
                Eigen::Matrix3d skew_com = SkewSymmetric(final_com);

                LinkInertiaData data;
                data.mass_matrix = current_aggregation.mass * Eigen::Matrix3d::Identity();
                data.cross_mass_matrix = current_aggregation.mass * skew_com;
                data.inertia_matrix = current_aggregation.inertia_world;

                link_inertia_map[current_group_name] = data;
            }

            // Reset aggregation
            current_aggregation = AggregatedInertia();
            current_link = next_link;
            current_group_name = current_link->name;
        }
    }

    ROS_INFO("Aggregated world inertia computed for %lu link groups.", link_inertia_map.size());

    // 7. Save to YAML
    std::string save_path = ros::package::getPath("smm_synthesis") + "/config/yaml/" + output_filename;

    YAML::Emitter out;
    out << YAML::BeginMap;
    for (const auto& entry : link_inertia_map)
    {
        const std::string& link_name = entry.first;
        const LinkInertiaData& data = entry.second;

        // Build 6x6 spatial inertia matrix
        Eigen::Matrix<double, 6, 6> spatial_inertia = Eigen::Matrix<double, 6, 6>::Zero();
        spatial_inertia.block<3,3>(0,0) = data.mass_matrix;
        //spatial_inertia.block<3,3>(0,3) = -data.cross_mass_matrix.transpose();
        spatial_inertia.block<3,3>(0,3) = -data.cross_mass_matrix;
        spatial_inertia.block<3,3>(3,0) = data.cross_mass_matrix;
        spatial_inertia.block<3,3>(3,3) = data.inertia_matrix;

        // Print nicely
        std::cout << "\nSpatial Inertia Matrix for Link Group: " << link_name << "\n";
        std::cout << spatial_inertia << "\n";

        // Save to YAML
        out << YAML::Key << link_name << YAML::Value << YAML::BeginMap;

        out << YAML::Key << "mass_matrix" << YAML::Value << YAML::BeginSeq;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                out << data.mass_matrix(i,j);
        out << YAML::EndSeq;

        out << YAML::Key << "cross_mass_matrix" << YAML::Value << YAML::BeginSeq;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                out << data.cross_mass_matrix(i,j);
        out << YAML::EndSeq;

        out << YAML::Key << "inertia_matrix" << YAML::Value << YAML::BeginSeq;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                out << data.inertia_matrix(i,j);
        out << YAML::EndSeq;

        out << YAML::EndMap;
    }

    std::ofstream fout(save_path);
    fout << out.c_str();
    fout.close();

    ROS_INFO_STREAM("Aggregated world inertias saved to: " << save_path);

    ros::spin();
    return 0;
}
