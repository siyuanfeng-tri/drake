#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/manipulator_streaming_plan.h"

#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

template <typename T>
ManipulatorStreamingPlan<T>::ManipulatorStreamingPlan() {}

template <typename T>
void ManipulatorStreamingPlan<T>::InitializeGenericPlanDerived(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) {
  const int dim = robot_status.position().size();
  q_d_ = robot_status.position();
  v_d_ = VectorX<T>::Zero(dim);
  vd_d_ = VectorX<T>::Zero(dim);
}

template <typename T>
void ManipulatorStreamingPlan<T>::HandlePlanMessageGenericPlanDerived(
    const HumanoidStatus& robot_stauts, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
    const void* message_bytes, int message_length) {}

template <typename T>
void ManipulatorStreamingPlan<T>::ExecutePlanGenericPlanDerived(
    const HumanoidStatus& robot_stauts, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) {}

template <typename T>
void ManipulatorStreamingPlan<T>::UpdateQpInputGenericPlanDerived(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
    QpInput* qp_input) const {
  VectorX<T> kp, kd;
  paramset.LookupDesiredDofMotionGains(&kp, &kd);

  VectorSetpoint<T> tracker(q_d_, v_d_, vd_d_, kp, kd);
  qp_input->mutable_desired_dof_motions().mutable_values() =
      tracker.ComputeTargetAcceleration(robot_status.position(),
                                        robot_status.velocity());
}

template <typename T>
GenericPlan<T>* ManipulatorStreamingPlan<T>::CloneGenericPlanDerived() const {
  ManipulatorStreamingPlan* clone = new ManipulatorStreamingPlan();

  clone->q_d_ = this->q_d_;
  clone->v_d_ = this->v_d_;
  clone->vd_d_ = this->vd_d_;

  return clone;
}

template class ManipulatorStreamingPlan<double>;

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
