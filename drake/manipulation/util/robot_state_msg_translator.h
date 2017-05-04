#pragma once

#include <unordered_map>
#include <utility>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/util/drakeGeometryUtil.h"
#include "drake/util/lcmUtil.h"

#include "lcmtypes/bot_core/robot_state_t.hpp"

namespace drake {
namespace manipulation {

class RobotStateLcmMessageTranslator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RobotStateLcmMessageTranslator)

  RobotStateLcmMessageTranslator(const RigidBodyTree<double>& robot);

  void ResizeMessage(bot_core::robot_state_t* msg) const;

  void DecodeMessageKinematics(const bot_core::robot_state_t& msg,
                               VectorX<double>* q, VectorX<double>* v) const;

  void EncodeMessageKinematics(const VectorX<double>& q,
                               const VectorX<double>& v,
                               bot_core::robot_state_t* msg) const;

  void DecodeMessageTorque(const bot_core::robot_state_t& msg,
                           VectorX<double>* torque) const;

  void EncodeMessageTorque(const VectorX<double>& torque,
                           bot_core::robot_state_t* msg) const;

  const RigidBodyTree<double>& get_robot() const { return robot_; }

  static bool CheckMessageVectorSize(const bot_core::robot_state_t& msg);

 private:
  /// Check that robot_state_t can unambiguously represent the RigidBodyTree's
  /// state. This method is intended to check preconditions inside a
  /// constructor's
  /// intitializer list.
  /// Aborts when the robot is not compatible with robot_state_t.
  /// @param robot a RigidBodyTree.
  /// @return the same RigidBodyTree that was passed in.
  static const RigidBodyTree<double>& CheckTreeIsRobotStateLcmTypeCompatible(
      const RigidBodyTree<double>& robot);

  const RigidBodyTree<double>& robot_;
  const RigidBody<double>* const floating_base_;

  std::unordered_map<std::string, int> joint_name_to_q_index_;
  std::unordered_map<std::string, int> joint_name_to_v_index_;
  std::unordered_map<std::string, int> joint_name_to_actuator_index_;
};

}  // namespace manipulation
}  // namespace drake
