#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <fstream>
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    ros::init(argc, argv, "pseudo_angle_extractor");
    ros::NodeHandle nh("~");

    // Define file paths
    std::string input_path = ros::package::getPath("smm_synthesis") + "/config/yaml/assembly.yaml";
    std::string output_path = ros::package::getPath("smm_synthesis") + "/config/yaml/q_pj_anat.yaml";

    // Load YAML file
    YAML::Node root;
    try {
        root = YAML::LoadFile(input_path);
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Failed to load assembly.yaml: " << e.what());
        return -1;
    }

    // Extract s-parameters
    int s2 = root["s2"].as<int>();
    int s3 = root["s3"].as<int>();
    int s5 = root["s5"].as<int>();
    int s6 = root["s6"].as<int>();

    // Extract all 4 angles
    std::vector<float> all_angles = {
        root["pseudo1_angle"].as<float>(),
        root["pseudo2_angle"].as<float>(),
        root["pseudo3_angle"].as<float>(),
        root["pseudo4_angle"].as<float>()
    };

    // Filter valid angles
    std::vector<float> filtered_angles;
    if (s2 != 9) filtered_angles.push_back(all_angles[0]);
    if (s3 != 9) filtered_angles.push_back(all_angles[1]);
    if (s5 != 9) filtered_angles.push_back(all_angles[2]);
    if (s6 != 9) filtered_angles.push_back(all_angles[3]);

    // Emit YAML
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "pseudo_angles" << YAML::Value << YAML::BeginSeq;
    for (float angle : filtered_angles)
        out << angle;
    out << YAML::EndSeq;
    out << YAML::EndMap;

    // Save output
    std::ofstream fout(output_path);
    fout << out.c_str();
    fout.close();

    ROS_INFO_STREAM("Filtered pseudo angles saved to: " << output_path);
    return 0;
}
