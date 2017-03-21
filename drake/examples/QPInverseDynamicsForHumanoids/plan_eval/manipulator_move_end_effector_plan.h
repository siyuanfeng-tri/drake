#pragma once

#include <string>

#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/generic_plan.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * A concrete plan that sets up a Cartesian tracking objective for the end
 * effector.
 */
template <typename T>
class ManipulatorMoveEndEffectorPlan : public GenericPlan<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ManipulatorMoveEndEffectorPlan<T>)

  ManipulatorMoveEndEffectorPlan() {}

 private:
  GenericPlan<T>* CloneGenericPlanDerived() const;

  // @p alias_groups must have a singleton body group name that matches
  // kEndEffectorAliasGroupName.
  void InitializeGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups);

  void ExecutePlanGenericPlanDerived(
      const HumanoidStatus& robot_stauts,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) {}

  // @p alias_groups must have a singleton body group name that matches
  // kEndEffectorAliasGroupName.
  // The lcm message passed in @p message_bytes needs to be of type
  // lcmt_manipulator_plan_move_end_effector
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

  static constexpr char kEndEffectorAliasGroupName[] = "end_effector";
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
