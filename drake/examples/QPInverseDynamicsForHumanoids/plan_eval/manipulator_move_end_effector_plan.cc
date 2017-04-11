#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/manipulator_move_end_effector_plan.h"

#include <vector>

#include "drake/lcmt_manipulator_plan_move_end_effector.hpp"
#include "drake/util/lcmUtil.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

template <typename T>
void ManipulatorMoveEndEffectorPlan<T>::InitializeGenericPlanDerived(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) {
  // Knots are constant, the second time doesn't matter.
  const std::vector<T> times = {robot_status.time(), robot_status.time() + 1};
  const RigidBody<T>* ee_body =
      alias_groups.get_body(kEndEffectorAliasGroupName);
  Isometry3<T> ee_pose = robot_status.robot().CalcBodyPoseInWorldFrame(
      robot_status.cache(), *ee_body);

  PiecewiseCartesianTrajectory<T> ee_traj =
      PiecewiseCartesianTrajectory<T>::MakeCubicLinearWithEndLinearVelocity(
          times, {ee_pose, ee_pose},
          Vector3<double>::Zero(), Vector3<double>::Zero());
  this->set_body_trajectory(ee_body, ee_traj);
}

template <typename T>
void ManipulatorMoveEndEffectorPlan<T>::HandlePlanMessageGenericPlanDerived(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
    const void* message_bytes, int message_length) {
  // Tries to decode as a lcmt_manipulator_plan_move_end_effector message.
  lcmt_manipulator_plan_move_end_effector msg;
  int consumed = msg.decode(message_bytes, 0, message_length);
  DRAKE_DEMAND(consumed == message_length);

  DRAKE_DEMAND(static_cast<size_t>(msg.num_steps) == msg.utimes.size() &&
               static_cast<size_t>(msg.num_steps) == msg.poses.size());
  DRAKE_DEMAND(msg.num_steps >= 0);

  if (msg.num_steps == 0) return;

  std::vector<T> times;
  std::vector<Isometry3<T>> poses;

  const RigidBody<T>* ee_body =
      alias_groups.get_body(kEndEffectorAliasGroupName);

  // If the first keyframe does not start immediately (its time > 0), we start
  // from the current desired pose.
  if (msg.utimes.front() != 0) {
    times.push_back(robot_status.time());
    poses.push_back(
        this->get_body_trajectory(ee_body).get_pose(robot_status.time()));
  }

  for (int i = 0; i < msg.num_steps; i++) {
    times.push_back(robot_status.time() + static_cast<T>(msg.utimes[i]) / 1e6);
    poses.push_back(DecodePose(msg.poses[i]));
  }

  // TODO(siyuan): use msg.order_of_interpolation.
  PiecewiseCartesianTrajectory<T> ee_traj =
      PiecewiseCartesianTrajectory<T>::MakeCubicLinearWithEndLinearVelocity(
          times, poses, Vector3<double>::Zero(), Vector3<double>::Zero());

  this->set_body_trajectory(ee_body, ee_traj);
}

template <typename T>
GenericPlan<T>* ManipulatorMoveEndEffectorPlan<T>::CloneGenericPlanDerived()
    const {
  return new ManipulatorMoveEndEffectorPlan();
}

template class ManipulatorMoveEndEffectorPlan<double>;

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
