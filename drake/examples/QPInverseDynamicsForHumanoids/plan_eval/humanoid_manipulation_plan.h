#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/humanoid_plan.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

template <typename T>
class HumanoidManipulationPlan : public HumanoidPlan<T> {
 public:
  HumanoidManipulationPlan() {}

 private:
  HumanoidPlan<T>* CloneHumanoidPlanDerived() const;

  void InitializeHumanoidPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups);

  void HandlePlanMessageGenericPlanDerived(
      const HumanoidStatus& robot_stauts,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      const void* message_bytes, int message_length);

  void ExecutePlanGenericPlanDerived(
      const HumanoidStatus& robot_stauts,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups);
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
