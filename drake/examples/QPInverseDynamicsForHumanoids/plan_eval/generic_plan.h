#pragma once

#include <memory>
#include <set>
#include <unordered_map>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/param_parsers/param_parser.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/param_parsers/rigid_body_tree_alias_groups.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/plan_eval_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller_common.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * This class represents a plan or equivalently a behavior. This class serves
 * as a bridge between a high level planner and a low level controller. It is
 * responsible for generating high frequency / dense inputs that are compatible
 * with the low level controller from the outputs from the high level planner.
 * Here is a concrete example: A motion planner generates a list of desired
 * postures to be tracked. A simple implementation of this class can be
 * interpolating those postures to compute position and velocity set points
 * for a low lever PID controller at high rate.
 */
template <typename T>
class GenericPlan {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GenericPlan)

  GenericPlan() {}

  /**
   * Returns a unique pointer to a copy of this instance. Derived classes need
   * to implement CloneGenericPlanDerived().
   */
  std::unique_ptr<GenericPlan<T>> Clone() const;

  virtual ~GenericPlan() {}

  /**
   * Initializes this plan. Derived classes need to implement custom behaviors
   * in InitializeGenericPlanDerived().
   * @param robot_status Current status of the robot.
   * @param paramset Parameters.
   * @param alias_groups Topological information of the robot.
   */
  void Initialize(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups);

  /**
   * Control logic should be implemented in this function. This function is
   * intended to be called in a main loop. Derived classes need to implement
   * custom behaviors in ExecutePlanGenericPlanDerived().
   * @param robot_status Current status of the robot.
   * @param paramset Parameters.
   * @param alias_groups Topological information of the robot.
   */
  void ExecutePlan(
      const HumanoidStatus& robot_stauts,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups);

  /**
   * Handles a discrete command in the form of a Lcm message (e.g. footstep
   * plan for a walking controller or a sequence of joint angles for a
   * manipulator). Derived classes need to implement custom behaviors in
   * HandlePlanMessageGenericPlanDerived().
   * @param robot_status Current status of the robot.
   * @param paramset Parameters.
   * @param alias_groups Topological information of the robot.
   * @param message_bytes Pointer to the raw message message. Derived classes
   * need to decode this explicitly.
   * @param message_length Number of bytes in the message.
   */
  void HandlePlanMessage(
      const HumanoidStatus& robot_stauts,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      const void* message_bytes, int message_length);

  /**
   * Generates a QpInput. Derived classes need to implement custom behaviors in
   * UpdateQpInputGenericPlanDerived().
   * @param robot_status Current status of the robot.
   * @param paramset Parameters.
   * @param alias_groups Topological information of the robot.
   * @param[out] qp_input Output for QpInput.
   */
  void UpdateQpInput(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      QpInput* qp_input) const;

  /**
   * Returns the current planned contact state.
   */
  const ContactState& get_contact_state() const { return contact_state_; }

  /**
   * Returns a map of all Cartesian trajectories.
   */
  const std::unordered_map<const RigidBody<T>*,
                           PiecewiseCartesianTrajectory<T>>&
  get_body_trajectories() const {
    return body_trajectories_;
  }

  /**
   * Returns the Cartesian trajectory for @p body.
   */
  const PiecewiseCartesianTrajectory<T>& get_body_trajectory(
      const RigidBody<T>* body) const {
    return body_trajectories_.at(body);
  }

  /**
   * Returns trajectory for all degrees of freedom.
   */
  const PiecewiseCubicTrajectory<T>& get_dof_trajectory() const {
    return dof_trajectory_;
  }

  /**
   * Returns true if there is a Cartesian trajectory for @p body.
   */
  const bool has_body_trajectory(const RigidBody<T>* body) const {
    return body_trajectories_.find(body) != body_trajectories_.end();
  }

 protected:
  virtual GenericPlan<T>* CloneGenericPlanDerived() const = 0;

  virtual void InitializeGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) = 0;

  virtual void ExecutePlanGenericPlanDerived(
      const HumanoidStatus& robot_stauts,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) = 0;

  virtual void HandlePlanMessageGenericPlanDerived(
      const HumanoidStatus& robot_stauts,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      const void* message_bytes, int message_length) = 0;

  virtual void UpdateQpInputGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      QpInput* qp_input) const = 0;

  /**
   * Sets the planned contact state to @p contact_state.
   */
  void UpdateContactState(const ContactState& contact_state) {
    contact_state_ = contact_state;
  }

  /**
   * Adds a Cartesian trajectory for @p body.
   */
  void set_body_trajectory(const RigidBody<T>* body,
                           const PiecewiseCartesianTrajectory<T>& traj) {
    auto it = body_trajectories_.find(body);
    if (it != body_trajectories_.end()) {
      body_trajectories_.erase(it);
    }
    body_trajectories_.emplace(body, traj);
  }

  /**
   * Removes a Cartesian trajectory for @p body.
   */
  const void remove_body_trajectory(const RigidBody<T>* body) {
    body_trajectories_.erase(body);
  }

  /**
   * Sets dof trajectory to @p traj.
   */
  void set_dof_trajectory(const PiecewiseCubicTrajectory<T>& traj) {
    dof_trajectory_ = traj;
  }

 private:
  ContactState contact_state_;
  PiecewiseCubicTrajectory<T> dof_trajectory_;
  std::unordered_map<const RigidBody<T>*, PiecewiseCartesianTrajectory<T>>
      body_trajectories_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
