#include <chrono>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <urdf/model.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

// Simple helper: KDL::Frame -> 4x4 Eigen
Eigen::Matrix4d KDLFrameToEigen(const KDL::Frame &frame)
{
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            mat(i, j) = frame.M(i, j);
        }
    }
    mat(0, 3) = frame.p.x();
    mat(1, 3) = frame.p.y();
    mat(2, 3) = frame.p.z();
    return mat;
}

// Mass aggregation for one "link group" (between two non-fixed joints)
struct AggregatedMass
{
    double mass = 0.0;
    Eigen::Vector3d weighted_world_com = Eigen::Vector3d::Zero();
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("com_extractor_kdl_ndof");

    // 1) Parameters
    node->declare_parameter<std::string>("output_file", "");
    node->declare_parameter<std::string>("robot_description", "");
    node->declare_parameter<std::string>("tip_link", "tcp");
    node->declare_parameter<std::string>("base_link_name", "base_link");

    std::string output_file;
    std::string robot_description;
    std::string tip_link;
    std::string base_link_name;

    node->get_parameter("output_file", output_file);
    node->get_parameter("robot_description", robot_description);
    node->get_parameter("tip_link", tip_link);
    node->get_parameter("base_link_name", base_link_name);

    if (output_file.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "Parameter 'output_file' is empty.");
        return 1;
    }
    if (robot_description.empty())
    {
        RCLCPP_ERROR(node->get_logger(),
                     "Parameter 'robot_description' is empty. "
                     "Make sure it is set in the launch file.");
        return 1;
    }

    // Small delay (optional, similar to your other nodes)
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(node->get_logger(), "Starting n-DoF CoM extraction...");

    // 2) Build URDF model
    urdf::Model model;
    if (!model.initString(robot_description))
    {
        RCLCPP_ERROR(node->get_logger(),
                     "Failed to parse URDF from robot_description parameter.");
        return 1;
    }

    // 3) Build KDL tree
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to construct KDL tree.");
        rclcpp::shutdown();
        return 1;
    }

    // Root of the KDL tree (usually "world")
    const std::string world_frame = kdl_tree.getRootSegment()->first;
    RCLCPP_INFO(node->get_logger(), "KDL tree root frame: '%s'", world_frame.c_str());
    RCLCPP_INFO(node->get_logger(), "Using tip_link: '%s'", tip_link.c_str());
    RCLCPP_INFO(node->get_logger(), "Base link for CoM groups: '%s'", base_link_name.c_str());

    // 4) Get KDL chain from world_frame to tip_link
    KDL::Chain chain;
    if (!kdl_tree.getChain(world_frame, tip_link, chain))
    {
        RCLCPP_ERROR(node->get_logger(),
                     "Failed to construct KDL chain from '%s' to '%s'.",
                     world_frame.c_str(), tip_link.c_str());
        rclcpp::shutdown();
        return 1;
    }

    const unsigned int n_segments = chain.getNrOfSegments();
    if (n_segments == 0)
    {
        RCLCPP_ERROR(node->get_logger(),
                     "KDL chain from '%s' to '%s' is empty.",
                     world_frame.c_str(), tip_link.c_str());
        rclcpp::shutdown();
        return 1;
    }

    // 5) Build list of link names along the chain:
    //    L0 = world_frame, L1, ..., L_M = tip_link
    std::vector<std::string> link_names(n_segments + 1);
    link_names[0] = world_frame;
    for (unsigned int i = 0; i < n_segments; ++i)
    {
        link_names[i + 1] = chain.getSegment(i).getName();
    }

    // 6) Precompute world->link transforms for each link L_i
    std::vector<KDL::Frame> world_T_link(n_segments + 1);
    world_T_link[0] = KDL::Frame::Identity(); // at world_frame
    for (unsigned int i = 0; i < n_segments; ++i)
    {
        const KDL::Segment &seg = chain.getSegment(i);
        world_T_link[i + 1] = world_T_link[i] * seg.pose(0.0);
    }

    // 7) Aggregate masses along the chain similar to your 3-DoF logic:
    //    - start at L1
    //    - each time we hit a non-fixed joint (between L_i and L_{i+1})
    //      we close the current group and start a new one at L_{i+1}.
    std::vector<AggregatedMass> chain_groups;
    std::vector<int> group_start_indices; // which L_i each group starts at

    AggregatedMass current_group;
    int current_group_start_idx = 1; // we start at L1
    const int M = static_cast<int>(n_segments);

    for (int i = 1; i <= M; ++i)
    {
        const std::string &link_name = link_names[i];
        urdf::LinkConstSharedPtr link = model.getLink(link_name);

        // 7.1 accumulate this link's mass into current group
        if (link && link->inertial)
        {
            double m = link->inertial->mass;
            if (m > 0.0)
            {
                // Inertial origin relative to link frame
                const urdf::Vector3 &pos = link->inertial->origin.position;

                // World frame of this link
                const KDL::Frame &w_T_link = world_T_link[i];

                // CoM in world frame = w_T_link * pos
                KDL::Vector com_world_kdl = w_T_link * KDL::Vector(pos.x, pos.y, pos.z);
                Eigen::Vector3d com_world(
                    com_world_kdl.x(),
                    com_world_kdl.y(),
                    com_world_kdl.z());

                current_group.mass += m;
                current_group.weighted_world_com += m * com_world;
            }
        }

        // 7.2 If this is the last link, close the group and break
        if (i == M)
        {
            if (current_group.mass > 0.0)
            {
                chain_groups.push_back(current_group);
                group_start_indices.push_back(current_group_start_idx);
            }
            break;
        }

        // 7.3 Check the joint between L_i and L_{i+1}
        const KDL::Joint &child_joint = chain.getSegment(i).getJoint();
        const auto joint_type = child_joint.getType();

        bool is_fixed = (joint_type == KDL::Joint::None);

        if (!is_fixed)
        {
            // Non-fixed joint: close current group and start a new one
            if (current_group.mass > 0.0)
            {
                chain_groups.push_back(current_group);
                group_start_indices.push_back(current_group_start_idx);
            }
            current_group = AggregatedMass();
            current_group_start_idx = i + 1; // next link is new group's start
        }
        // else: still inside same group
    }

    if (chain_groups.empty())
    {
        RCLCPP_ERROR(node->get_logger(),
                     "No groups with mass were found along the chain from '%s' to '%s'.",
                     world_frame.c_str(), tip_link.c_str());
        rclcpp::shutdown();
        return 1;
    }

    // 8) Drop groups BEFORE base_link_name, and count how many remain.
    int start_group = -1;
    for (size_t g = 0; g < chain_groups.size(); ++g)
    {
        int idx = group_start_indices[g];
        if (idx >= 0 && idx <= M && link_names[idx] == base_link_name)
        {
            start_group = static_cast<int>(g);
            break;
        }
    }

    if (start_group == -1)
    {
        RCLCPP_WARN(node->get_logger(),
                    "No group starts at base_link_name='%s'. "
                    "Using first group as gsl00.",
                    base_link_name.c_str());
        start_group = 0;
    }

    const int groups_to_use = static_cast<int>(chain_groups.size()) - start_group;
    if (groups_to_use <= 0)
    {
        RCLCPP_ERROR(node->get_logger(),
                     "No groups remain after filtering before base_link_name='%s'.",
                     base_link_name.c_str());
        rclcpp::shutdown();
        return 1;
    }

    if (groups_to_use < 3 || groups_to_use > 6)
    {
        RCLCPP_WARN(node->get_logger(),
                    "Detected %d CoM groups after '%s'. "
                    "Expected between 3 and 6 for SMM nDoF, continuing anyway.",
                    groups_to_use, base_link_name.c_str());
    }
    else
    {
        RCLCPP_INFO(node->get_logger(),
                    "Will output %d CoM groups (3–6 DoF range).", groups_to_use);
    }

    // 9) Build final map: gsl00, gsl10, gsl20, ... (store COM as [x, y, z] in world frame)
    std::map<std::string, Eigen::Vector3d> com_positions;

    for (int k = 0; k < groups_to_use; ++k)
    {
        int g_idx = start_group + k;
        const AggregatedMass &agg = chain_groups[g_idx];

        if (agg.mass <= 0.0)
        {
            RCLCPP_WARN(node->get_logger(),
                        "Group %d has zero mass, skipping.", g_idx);
            continue;
        }

        Eigen::Vector3d com_world = agg.weighted_world_com / agg.mass;

        std::string internal_name = "gsl" + std::to_string(k) + "0";
        com_positions[internal_name] = com_world;

        int link_idx = group_start_indices[g_idx];
        RCLCPP_INFO(node->get_logger(),
                    "Group %d (start link='%s') → '%s', mass = %f, CoM = [%f, %f, %f]",
                    g_idx, link_names[link_idx].c_str(),
                    internal_name.c_str(), agg.mass,
                    com_world(0), com_world(1), com_world(2));
    }

    // 10) Write YAML: gslXX → [x, y, z]
    YAML::Emitter out;
    out << YAML::BeginMap;
    for (const auto &kv : com_positions)
    {
        out << YAML::Key << kv.first << YAML::Value << YAML::BeginSeq;
        out << kv.second(0) << kv.second(1) << kv.second(2);
        out << YAML::EndSeq;
    }
    out << YAML::EndMap;


    std::ofstream fout(output_file);
    if (!fout.is_open())
    {
        RCLCPP_ERROR(node->get_logger(),
                     "Failed to open output file: '%s'", output_file.c_str());
        rclcpp::shutdown();
        return 1;
    }
    fout << out.c_str();
    fout.close();

    RCLCPP_INFO(node->get_logger(),
                "CoM frames saved to: %s", output_file.c_str());

    rclcpp::shutdown();
    return 0;
}
