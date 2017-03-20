#pragma once

#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/generic_plan.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

template <typename T>
class ManipulatorStreamingPlan : public GenericPlan<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ManipulatorStreamingPlan<T>)

  ManipulatorStreamingPlan();

  const VectorX<T>& get_reference_position() { return q_d_; }
  const VectorX<T>& get_reference_velocity() { return v_d_; }
  const VectorX<T>& get_reference_acceleration() { return vd_d_; }
  VectorX<T>& get_mutable_reference_position() { return q_d_; }
  VectorX<T>& get_mutable_reference_velocity() { return v_d_; }
  VectorX<T>& get_mutable_reference_acceleration() { return vd_d_; }

 private:
  GenericPlan<T>* CloneGenericPlanDerived() const;

  void InitializeGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups);

  void ExecutePlanGenericPlanDerived(
      const HumanoidStatus& robot_stauts,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups);

  void HandlePlanMessageGenericPlanDerived(
      const HumanoidStatus& robot_stauts,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      const void* message_bytes, int message_length);

  void UpdateQpInputGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      QpInput* qp_input) const;

  VectorX<T> q_d_;
  VectorX<T> v_d_;
  VectorX<T> vd_d_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
