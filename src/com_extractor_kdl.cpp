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

// Run for test:
// $ rosrun smm_synthesis com_extractor_kdl 
// OR
// $ roslaunch smm_synthesis extract_zero_com.launch

// Struct to hold mass and world-frame weighted CoM
struct AggregatedMass {
double mass;
Eigen::Vector3d weighted_world_com;
AggregatedMass() : mass(0.0), weighted_world_com(Eigen::Vector3d::Zero()) {}
};

Eigen::Vector3d KDLFrameToPosition(const KDL::Frame& frame) {
return Eigen::Vector3d(frame.p.x(), frame.p.y(), frame.p.z());
}

int main(int argc, char** argv)
{
ros::init(argc, argv, "com_extractor_kdl");
ros::NodeHandle nh("~");

    // Get output filename
    std::string output_filename;
    if (!nh.getParam("output_file", output_filename)) {
        ROS_ERROR("No output_file parameter given.");
        return -1;
    }

    // Load URDF
    urdf::Model model;
    if (!model.initParam("robot_description")) {
        ROS_ERROR("Failed to load URDF from parameter server.");
        return -1;
    }

    // Parse KDL tree
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)) {
        ROS_ERROR("Failed to construct KDL tree.");
        return -1;
    }

    std::string world_frame = kdl_tree.getRootSegment()->first;
    ROS_INFO_STREAM("Using world frame: " << world_frame);

    // Output map: frame_name (mapped) -> world CoM
    std::map<std::string, Eigen::Vector3d> aggregated_coms;

    // Aggregation setup
    AggregatedMass current_aggregation;
    std::string current_group_name;

    // Link traversal
    urdf::LinkConstSharedPtr current_link = model.getRoot();
    if (!current_link) {
        ROS_ERROR("Null root link.");
        return -1;
    }

    current_link = current_link->child_links.empty() ? nullptr : current_link->child_links[0];
    if (!current_link) {
        ROS_ERROR("World has no child links.");
        return -1;
    }

    current_group_name = current_link->name;

    // Mapping from frame name to key
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
            ROS_ERROR_STREAM("Failed to get chain to " << current_link->name);
            break;
        }

        KDL::Frame world_T_link = KDL::Frame::Identity();
        for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i)
            world_T_link = world_T_link * chain.getSegment(i).pose(0.0);

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

        // Stop if this is a leaf
        if (current_link->child_joints.empty()) {
            if (current_aggregation.mass > 0.0 && name_map.count(current_group_name)) {
                Eigen::Vector3d final_com = current_aggregation.weighted_world_com / current_aggregation.mass;
                aggregated_coms[name_map[current_group_name]] = final_com;
            }
            break;
        }

        // Traverse next
        urdf::JointConstSharedPtr child_joint = current_link->child_joints[0];
        urdf::LinkConstSharedPtr next_link = current_link->child_links[0];

        if (!child_joint || !next_link) {
            ROS_ERROR_STREAM("Invalid child joint or link at: " << current_link->name);
            break;
        }

        if (child_joint->type == urdf::Joint::FIXED) {
            current_link = next_link;
        } else {
            if (current_aggregation.mass > 0.0 && name_map.count(current_group_name)) {
                Eigen::Vector3d final_com = current_aggregation.weighted_world_com / current_aggregation.mass;
                aggregated_coms[name_map[current_group_name]] = final_com;
            }

            current_aggregation = AggregatedMass();
            current_link = next_link;
            current_group_name = current_link->name;
        }
    }

    // Output YAML
    std::string save_path = ros::package::getPath("smm_synthesis") + "/config/yaml/" + output_filename;
    YAML::Emitter out;
    out << YAML::BeginMap;
    for (const auto& kv : aggregated_coms) {
        out << YAML::Key << kv.first << YAML::Value << YAML::BeginSeq;
        out << kv.second.x() << kv.second.y() << kv.second.z();
        out << YAML::EndSeq;
    }
    out << YAML::EndMap;

    std::ofstream fout(save_path);
    fout << out.c_str();
    fout.close();

    ROS_INFO_STREAM("World-frame CoMs saved to: " << save_path);
    return 0;
}
