#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/manipulator_move_joints_plan.h"

#include <vector>

#include "robotlocomotion/robot_plan_t.hpp"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

template <typename T>
void ManipulatorMoveJointsPlan<T>::HandlePlanMessageGenericPlanDerived(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
    const void* message_bytes, int message_length) {
  // Tries to decode as a robotlocomotion::robot_plan_t message.
  robotlocomotion::robot_plan_t msg;
  int consumed = msg.decode(message_bytes, 0, message_length);
  DRAKE_DEMAND(consumed == message_length);

  int length = static_cast<int>(msg.plan.size());
  const int dim = robot_status.robot().get_num_positions();
  if (length == 0) return;

  std::vector<MatrixX<T>> dof_knots;
  std::vector<T> times;

  // Initial desired velocity.
  VectorX<T> vel0_d = VectorX<T>::Zero(dim);

  // If the first keyframe does not start immediately (its time > 0), we start
  // from the current desired position and velocity.
  if (msg.plan.front().utime != 0) {
    dof_knots.push_back(
        this->get_dof_trajectory().get_position(robot_status.time()));
    times.push_back(robot_status.time());
    vel0_d = this->get_dof_trajectory().get_velocity(robot_status.time());
  }

  for (const bot_core::robot_state_t& keyframe : msg.plan) {
    DRAKE_DEMAND(keyframe.num_joints == dim);
    times.push_back(robot_status.time() + static_cast<T>(keyframe.utime) / 1e6);
    dof_knots.push_back(MatrixX<T>(dim, 1));
    for (int i = 0; i < keyframe.num_joints; i++) {
      dof_knots.back()(i, 0) = keyframe.joint_position.at(i);
    }
  }

  PiecewisePolynomial<T> pos_traj =
      PiecewisePolynomial<T>::Cubic(
          times, dof_knots, vel0_d, VectorX<T>::Zero(dim));
  this->set_dof_trajectory(PiecewiseCubicTrajectory<T>(pos_traj));
}

template <typename T>
GenericPlan<T>* ManipulatorMoveJointsPlan<T>::CloneGenericPlanDerived() const {
  return new ManipulatorMoveJointsPlan();
}

template class ManipulatorMoveJointsPlan<double>;

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
