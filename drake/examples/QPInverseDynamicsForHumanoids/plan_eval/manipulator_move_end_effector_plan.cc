#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/manipulator_move_end_effector_plan.h"

#include "drake/lcmt_manipulator_plan_move_end_effector.hpp"
#include "drake/util/lcmUtil.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

template <typename T>
ManipulatorMoveEndEffectorPlan<T>::ManipulatorMoveEndEffectorPlan() {}

template <typename T>
void ManipulatorMoveEndEffectorPlan<T>::InitializeGenericPlanDerived(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) {
  // Knots are constant, the second time doesn't matter.
  const std::vector<T> times = {robot_status.time(), robot_status.time() + 1};
  const MatrixX<double> q_d = robot_status.position();
  this->set_dof_trajectory(
      PiecewiseCubicTrajectory<T>(
          PiecewisePolynomial<T>::ZeroOrderHold(times, {q_d, q_d})));

  std::string ee_name = "end_effector";
  const RigidBody<T>* ee_body = alias_groups.get_body(ee_name);
  Isometry3<T> ee_pose = robot_status.robot().CalcBodyPoseInWorldFrame(
        robot_status.cache(), *ee_body);
  this->set_body_trajectory(
      ee_body, PiecewiseCartesianTrajectory<T>(
          times, {ee_pose, ee_pose}, true));
}

template <typename T>
void ManipulatorMoveEndEffectorPlan<T>::HandlePlanMessageGenericPlanDerived(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
    const void* message_bytes, int message_length) {
  lcmt_manipulator_plan_move_end_effector msg;
  int consumed = msg.decode(message_bytes, 0, message_length);
  DRAKE_DEMAND(consumed == message_length);

  std::vector<T> times(msg.num_steps);
  std::vector<Isometry3<T>> poses(msg.num_steps);
  for (int i = 0; i < msg.num_steps; i++) {
    times[i] = robot_status.time() + msg.times[i];
    poses[i] = DecodePose(msg.poses[i]);
  }

  // TODO(siyuan): use msg.order_of_interpolation.
  PiecewiseCartesianTrajectory<T> ee_traj(times, poses, true);

  std::string ee_name = "end_effector";
  const RigidBody<T>* ee_body = alias_groups.get_body(ee_name);

  this->set_body_trajectory(ee_body, ee_traj);
}

template <typename T>
GenericPlan<T>* ManipulatorMoveEndEffectorPlan<T>::CloneGenericPlanDerived() const {
  return new ManipulatorMoveEndEffectorPlan();
}

template class ManipulatorMoveEndEffectorPlan<double>;

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
