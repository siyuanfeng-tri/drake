#include "drake/multibody/parser_yaml.h"

#include <string>
#include <vector>

namespace drake {
namespace parsers {

const std::string kJointGroupKeyWord = "joint_groups";
const std::string kBodyGroupKeyWord = "body_groups";

void ParseJointGroups(const YAML::Node& metadata,
                      RigidBodyTree<double>* robot) {
  if (!robot) return;

  YAML::Node joint_groups = metadata[kJointGroupKeyWord];
  for (auto group_it = joint_groups.begin(); group_it != joint_groups.end();
       ++group_it) {
    std::string group_name = group_it->first.as<std::string>();
    std::vector<std::string> joint_names =
        group_it->second.as<std::vector<std::string>>();

    std::vector<int> position_idx, velocity_idx;
    for (const std::string& joint_name : joint_names) {
      const RigidBody& body = *(robot->FindChildBodyOfJoint(joint_name));

      int q_start = body.get_position_start_index();
      int q_end = q_start + body.getJoint().get_num_positions();
      for (int idx = q_start; idx < q_end; ++idx) position_idx.push_back(idx);

      int v_start = body.get_velocity_start_index();
      int v_end = v_start + body.getJoint().get_num_velocities();
      for (int idx = v_start; idx < v_end; ++idx) velocity_idx.push_back(idx);
    }
    robot->AddPositionGroup(group_name, position_idx);
    robot->AddVelocityGroup(group_name, velocity_idx);
  }
}

void ParseBodyGroups(const YAML::Node& metadata, RigidBodyTree<double>* robot) {
  if (!robot) return;

  YAML::Node body_groups = metadata[kBodyGroupKeyWord];
  for (auto group_it = body_groups.begin(); group_it != body_groups.end();
       ++group_it) {
    std::string group_name = group_it->first.as<std::string>();
    std::vector<std::string> body_names =
        group_it->second.as<std::vector<std::string>>();

    robot->AddBodyGroup(group_name, body_names);
  }
}

}  // namespace parsers
}  // namespace drake
