#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/generic_plan.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

template <typename T>
std::unique_ptr<GenericPlan<T>> GenericPlan<T>::Clone() const {
  std::unique_ptr<GenericPlan<T>> clone(CloneGenericPlanDerived());
  clone->contact_state_ = this->contact_state_;
  clone->body_trajectories_ = this->body_trajectories_;

  return clone;
}

template <typename T>
void GenericPlan<T>::Initialize(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) {
  // Sets contact states sequence to empty from 0 to forever.
  ContactState empty;
  UpdateContactState(empty);

  // Clears all tracked body trajectories.
  body_trajectories_.clear();

  // Calls custom stuff.
  InitializeGenericPlanDerived(robot_status, paramset, alias_groups);
}

template <typename T>
void GenericPlan<T>::HandlePlanMessage(
    const HumanoidStatus& robot_status,
    const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
    const void* message_bytes, int message_length) {
  HandlePlanMessageGenericPlanDerived(robot_status, paramset, alias_groups,
      message_bytes, message_length);
}

template <typename T>
void GenericPlan<T>::ExecutePlan(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) {
  // Runs derived class' plan.
  ExecutePlanGenericPlanDerived(robot_status, paramset, alias_groups);
}

template <typename T>
void GenericPlan<T>::UpdateQpInput(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
    QpInput* qp_input) const {
  const ContactState& contact_state = get_contact_state();

  const std::vector<const RigidBody<T>*> contact_bodies(contact_state.begin(),
                                                        contact_state.end());

  std::vector<const RigidBody<T>*> tracked_bodies;
  for (const auto& tracked_body_pair : body_trajectories_) {
    tracked_bodies.push_back(tracked_body_pair.first);
  }

  // Sets up weights and modes.
  *qp_input =
      paramset.MakeQpInput(contact_bodies, tracked_bodies, alias_groups);

  // Do more updates, like update desired values etc.
  UpdateQpInputGenericPlanDerived(robot_status, paramset, alias_groups,
                                  qp_input);
}

template class GenericPlan<double>;

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
