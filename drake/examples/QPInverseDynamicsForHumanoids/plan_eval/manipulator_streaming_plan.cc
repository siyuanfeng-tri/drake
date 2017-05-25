#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/manipulator_streaming_plan.h"

#include <vector>

#include "drake/common/unused.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/lcmt_manipulator_plan_streaming.hpp"
#include "drake/util/drakeUtil.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

template <typename T>
void ManipulatorStreamingPlan<T>::InitializeGenericPlanDerived(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) {
  unused(paramset);  // TODO(jwnimmer-tri) This seems bad.

  desired_q_ = robot_status.position();
  desired_v_ = VectorX<T>::Zero(robot_status.robot().get_num_velocities());
  desired_vd_ = VectorX<T>::Zero(robot_status.robot().get_num_velocities());
}

template <typename T>
void ManipulatorStreamingPlan<T>::HandlePlanMessageGenericPlanDerived(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
    const void* message_bytes, int message_length) {
  unused(paramset);  // TODO(jwnimmer-tri) This seems bad.

  lcmt_manipulator_plan_streaming msg;
  int consumed = msg.decode(message_bytes, 0, message_length);
  DRAKE_DEMAND(consumed == message_length);

  if (msg.num_joints != robot_status.robot().get_num_positions()) {
    drake::log()->warn("dimension mismatch");
  }

  // TODO(siyuan): should do better error handling wrt bad plan message.
  DRAKE_DEMAND(msg.num_joints == robot_status.robot().get_num_positions());

  stdVectorToEigenVector(msg.desired_position, desired_q_);
  stdVectorToEigenVector(msg.desired_velocity, desired_v_);
  stdVectorToEigenVector(msg.desired_acceleration, desired_vd_);
}

template <typename T>
void ManipulatorStreamingPlan<T>::UpdateDofMotion(
    const HumanoidStatus& robot_status,
    const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>&,
    DesiredDofMotions* dof_motions) const {
  // Override the desired acceleration.
  VectorX<T> kp, kd;
  paramset.LookupDesiredDofMotionGains(&kp, &kd);
  VectorSetpoint<T> tracker(desired_q_, desired_v_, desired_vd_, kp, kd);

  dof_motions->mutable_values() =
      tracker.ComputeTargetAcceleration(robot_status.position(),
                                        robot_status.velocity());
}

template <typename T>
GenericPlan<T>* ManipulatorStreamingPlan<T>::CloneGenericPlanDerived()
    const {
  return new ManipulatorStreamingPlan(*this);
}

template class ManipulatorStreamingPlan<double>;

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
