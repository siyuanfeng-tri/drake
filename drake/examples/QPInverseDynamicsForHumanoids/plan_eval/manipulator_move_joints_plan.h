#pragma once

#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/generic_plan.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * A concrete plan that sets up a joint space tracking objective.
 */
template <typename T>
class ManipulatorMoveJointsPlan : public GenericPlan<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ManipulatorMoveJointsPlan<T>)

  ManipulatorMoveJointsPlan() {}

 private:
  GenericPlan<T>* CloneGenericPlanDerived() const;

  void InitializeGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) {}

  void ExecutePlanGenericPlanDerived(
      const HumanoidStatus& robot_stauts,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) {}

  // the lcm message passed in @p message_bytes needs to be of type
  // robotlocomotion::robot_plan_t.
  void HandlePlanMessageGenericPlanDerived(
      const HumanoidStatus& robot_stauts,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      const void* message_bytes, int message_length);

  void UpdateQpInputGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      QpInput* qp_input) const {}
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
