#pragma once

#include <string>

#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/generic_plan.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * A concrete plan that sets up a Cartesian tracking objective for the end
 * effector assuming no contacts. The joint space tracking objective is set
 * to track the measured joint position when Initialize() is called.
 * Note that joint position tracking can be turned off by setting the
 * position gains to zero in the parameters passed to UpdateQpInput.
 */
template <typename T>
class ManipulatorStreamingPlan : public GenericPlan<T> {
 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ManipulatorStreamingPlan)

 public:
  ManipulatorStreamingPlan() {}

 protected:
  /**
   * A desired stationary Cartesian spline is set up for the body whose alias
   * in @p alias_groups is kEndEffectorAliasGroupName.
   */
  void InitializeGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) override;

  void HandlePlanMessageGenericPlanDerived(
      const HumanoidStatus& robot_stauts,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      const void* message_bytes, int message_length) override;

 private:
  GenericPlan<T>* CloneGenericPlanDerived() const override;

  void UpdateDofMotion(const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      DesiredDofMotions* dof_motions) const override;

  VectorX<T> desired_q_;
  VectorX<T> desired_v_;
  VectorX<T> desired_vd_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
