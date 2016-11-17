#pragma once

#include <string>

#include "drake/multibody/rigid_body_tree.h"

#include "yaml-cpp/yaml.h"

namespace drake {
namespace parsers {

extern const std::string kJointGroupKeyWord;
extern const std::string kBodyGroupKeyWord;

/**
 * Parse @p robot's joint groups specified by @p metadata, which is loaded from
 * a YAML file.
 *
 * The config file is assumed to be formatted as:
 * <pre>
 * ...
 * joint_groups:
 *   group_name1:
 *     [joint_name1, joint_name2, ...]
 *   group_name2:
 *     [joint_name1, joint_name3, ...]
 *   ...
 * ...
 * </pre>
 * This function is looking for a top level keyword kJointGroupKeyWord
 * which contains individual groups consists of a vector of joint names.
 * The result is stored in @p robot, and can be accessed by calling
 * RigidBodyTree::get_position_group and RigidBodyTree::get_velocity_group.
 *
 * @param metadata, YAML node representing the config file.
 * @param robot, Pointer to the RigidBodyTree.
 */
void ParseJointGroups(const YAML::Node& metadata, RigidBodyTree<double>* robot);

/**
 * Parse @p robot's body groups specified by @p metadata, which is loaded from
 * a YAML file.
 *
 * The config file is assumed to be formatted as:
 * <pre>
 * ...
 * body_groups:
 *   group_name1:
 *     [body_name1, body_name2, ...]
 *   group_name2:
 *     [body_name1, body_name3, ...]
 *   ...
 * ...
 * </pre>
 * This function is looking for a top level keyword kBodyGroupKeyWord,
 * which contains individual groups consists of a vector of body names.
 * The result is stored in @p robot, and can be accessed by calling
 * RigidBodyTree::get_body_group.
 *
 * @param metadata, YAML node representing the config file.
 * @param robot, Pointer to the RigidBodyTree.
 */
void ParseBodyGroups(const YAML::Node& metadata, RigidBodyTree<double>* robot);

}  // namespace parsers
}  // namespace drake
