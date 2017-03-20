#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/manipulator_move_joints_plan.h"

#include "robotlocomotion/robot_plan_t.hpp"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

template <typename T>
ManipulatorMoveJointPlan<T>::ManipulatorMoveJointPlan() {}

template <typename T>
void ManipulatorMoveJointPlan<T>::InitializeGenericPlanDerived(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) {
  // Knots are constant, the second time doesn't matter.
  const std::vector<T> times = {robot_status.time(), robot_status.time() + 1};
  const MatrixX<double> q_d = robot_status.position();
  this->set_dof_trajectory(
      PiecewiseCubicTrajectory<T>(
          PiecewisePolynomial<T>::ZeroOrderHold(times, {q_d, q_d})));
}

template <typename T>
void ManipulatorMoveJointPlan<T>::HandlePlanMessageGenericPlanDerived(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
    const void* message_bytes, int message_length) {
  robotlocomotion::robot_plan_t msg;
  int consumed = msg.decode(message_bytes, 0, message_length);
  DRAKE_DEMAND(consumed == message_length);

  const int dim = robot_status.robot().get_num_positions();

  int length = static_cast<int>(msg.plan.size());
  std::vector<MatrixX<T>> dof_knots(length);
  std::vector<T> times(length);
  for (int f = 0; f < length; f++) {
    const bot_core::robot_state_t& keyframe = msg.plan[f];

    times[f] = robot_status.time() + static_cast<double>(keyframe.utime) / 1e6;
    dof_knots[f].resize(keyframe.num_joints, 1);
    DRAKE_DEMAND(keyframe.num_joints == dim);
    for (int i = 0; i < keyframe.num_joints; i++) {
      dof_knots[f](0, i) = keyframe.joint_position.at(i);
    }
  }

  MatrixX<T> zeros = VectorX<T>::Zero(dim);
  this->set_dof_trajectory(
      PiecewisePolynomial<T>::Cubic(times, dof_knots, zeros, zeros));
}

template <typename T>
GenericPlan<T>* ManipulatorMoveJointPlan<T>::CloneGenericPlanDerived() const {
  return new ManipulatorMoveJointPlan();
}

template class ManipulatorMoveJointPlan<double>;

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
