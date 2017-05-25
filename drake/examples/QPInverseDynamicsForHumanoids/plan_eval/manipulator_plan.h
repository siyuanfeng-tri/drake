#pragma once

#include <string>

#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/generic_plan.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/// This first version assumes the given plan is actually a cartesian move
/// plan without contacts.
template <typename T>
class ManipulatorPlan : public GenericPlan<T> {
 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ManipulatorPlan)

 public:
  ManipulatorPlan() {}

 protected:
  void InitializeGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) override;

  void HandlePlanMessageGenericPlanDerived(
      const HumanoidStatus& robot_stauts,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      const void* message_bytes, int message_length) override;

  void UpdateQpInputGenericPlanDerived(
      const HumanoidStatus&,
      const param_parsers::ParamSet&,
      const param_parsers::RigidBodyTreeAliasGroups<T>&,
      QpInput* qp_input) const override {
    qp_input->mutable_contact_information() = override_contacts_;
  }

  void ModifyPlanGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet&,
      const param_parsers::RigidBodyTreeAliasGroups<T>&) override;

  void MakeDebugMessage(
      const HumanoidStatus& robot_stauts,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      std::vector<uint8_t>* raw_bytes) const override;

 private:
  GenericPlan<T>* CloneGenericPlanDerived() const override;

  std::unordered_map<std::string, ContactInformation> override_contacts_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
