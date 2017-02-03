#pragma once

#include <memory>
#include <string>

#include "drake/examples/QPInverseDynamicsForHumanoids/param_parsers/param_parser.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/param_parsers/rigid_body_tree_alias_groups.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * A base block that generates qp input for the qp inverse dynamics controller.
 * This class must be extended to implement the desired behaviors. Due to its
 * discrete time nature, control is computed in DoCalcUnrestrictedUpdate(),
 * and the result is stored in its AbstractState. DoCalcOutput() merely copies
 * the latest result from its AbstractState and sends it through the output
 * port. Context's time must be properly maintained. The internal states of
 * the plan are also stored in its AbstractState, and can be modified in
 * DoCalcUnrestrictedUpdate().
 */
class DiscreteTimePlanEvalSystem : public systems::LeafSystem<double> {
 public:
  DiscreteTimePlanEvalSystem(const RigidBodyTree<double>& robot,
                             const std::string& alias_groups_file_name,
                             const std::string& param_file_name, double dt);

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  std::unique_ptr<systems::AbstractValue> AllocateOutputAbstract(
      const systems::OutputPortDescriptor<double>& descriptor) const override;

  std::unique_ptr<systems::AbstractState> AllocateAbstractState()
      const override {
    throw std::runtime_error(
        "Subclass need to implememt AllocateOutputAbstract.");
    return std::make_unique<systems::AbstractState>();
  }

  void DoCalcUnrestrictedUpdate(const systems::Context<double>& context,
                                systems::State<double>* state) const override {
    throw std::runtime_error(
        "Subclass need to implememt DoCalcUnrestrictedUpdate.");
  }

  /**
   * Set the set point for tracking.
   * @param q_d Desired generalized position.
   * @param state State that holds the plan.
   */
  void SetDesired(const VectorX<double>& q_d, systems::State<double>* state);

  /**
   * @return Port for the input: HumanoidStatus.
   */
  inline const systems::InputPortDescriptor<double>&
  get_input_port_humanoid_status() const {
    return get_input_port(input_port_index_humanoid_status_);
  }

  /**
   * @return Port for the output: QpInput.
   */
  inline const systems::OutputPortDescriptor<double>& get_output_port_qp_input()
      const {
    return get_output_port(output_port_index_qp_input_);
  }

 protected:
  template <typename PlanType>
  PlanType& get_mutable_plan(systems::State<double>* state) const {
    return state->get_mutable_abstract_state()
        ->get_mutable_abstract_state(abstract_state_plan_index_)
        .GetMutableValue<PlanType>();
  }

  QpInput& get_mutable_qp_input(systems::State<double>* state) const {
    return state->get_mutable_abstract_state()
        ->get_mutable_abstract_state(abstract_state_qp_input_index_)
        .GetMutableValue<QpInput>();
  }

  const RigidBodyTree<double>& robot_;
  const double control_dt_{2e-3};

  param_parsers::RigidBodyTreeAliasGroups<double> alias_groups_;
  param_parsers::ParamSet paramset_;

  int input_port_index_humanoid_status_{0};
  int output_port_index_qp_input_{0};
  const int abstract_state_qp_input_index_;
  const int abstract_state_plan_index_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
