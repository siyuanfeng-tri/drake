#pragma once

#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/generic_plan.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * A concrete plan that sets up a joint space tracking objective without any
 * contacts or body tracking objectives.
 */
template <typename T>
class ManipulatorMoveJointsPlan : public GenericPlan<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ManipulatorMoveJointsPlan<T>)

  ManipulatorMoveJointsPlan() {}

 protected:
  /**
   * This function assumes that the bytes in @p message_bytes encodes a valid
   * robotlocomotion::robot_plan_t message, from which the joint
   * positions (q_i) and timing (t_i) are extracted and used to construct a
   * cubic spline. Let t_now be the time in @p robot_stauts. If t_0 = 0
   * (first time stamp in @p message_bytes is zero), the spline is constructed
   * with PiecewisePolynomial::Cubic(t_i + t_now, q_i, 0, 0), which starts
   * immediately from q_0 regardless of what's the current planned desired
   * position. This introduces a discontinuity in the interpolated desired
   * position, and can cause the output and ultimately the robot motion
   * being "jumpy". If t_0 > 0, the spline is instead constructed with
   * PiecewisePolynomial::Cubic({0, t_i} + t_now, {q_d_now, q_i}, v_d_now, 0),
   * where q_d_now and v_d_now are the current desired position and velocity.
   * This results in a smoother transition to the new plan.
   */
  void HandlePlanMessageGenericPlanDerived(
      const HumanoidStatus& robot_stauts,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      const void* message_bytes, int message_length);

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

  void UpdateQpInputGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      QpInput* qp_input) const {}
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
