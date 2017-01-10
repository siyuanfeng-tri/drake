#pragma once

#include <memory>

#include "drake/examples/QPInverseDynamicsForHumanoids/param_parsers/param_parser.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * A simple PlanEval block that generates qp input for the qp inverse dynamics
 * controller.
 * The controller is set up to track a stationary fixed point assuming the
 * robot is in double support, and the desired set point is set by SetDesired.
 *
 * Conceptually, this block is a discrete time controller. In order to capture
 * the discrete time nature, control is performed in DoCalcUnrestrictedUpdate,
 * and the result is stored in AbstractState. DoCalcOutput merely copies the
 * latest result from the AbstractState and sends it through the output port.
 * Context's time must properly maintained. The internal states of the plan
 * are also stored in the AbstractState, and can be modified in
 * DoCalcUnrestrictedUpdate.
 *
 * Input: HumanoidStatus
 * Output: lcmt_qp_input
 */
class PlanEvalSystem : public systems::LeafSystem<double> {
 public:
  explicit PlanEvalSystem(const RigidBodyTree<double>& robot);

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  // This is used to trigger DoCalcUnrestrictedUpdate every control_dt_.
  void DoCalcNextUpdateTime(
      const systems::Context<double>& context,
      systems::UpdateActions<double>* actions) const override;

  void DoCalcUnrestrictedUpdate(const systems::Context<double>& context,
                                systems::State<double>* state) const override;

  std::unique_ptr<systems::SystemOutput<double>> AllocateOutput(
      const systems::Context<double>& context) const override;

  std::unique_ptr<systems::AbstractState> AllocateAbstractState()
      const override;

  /**
   * Sets the desired setpoint and initializes the State that contains the plan.
   * @param q Desired generalized position
   * @param state State in the Context
   */
  void SetDesired(const VectorX<double>& q, systems::State<double>* state);

  /**
   * @return Port for the input: HumanoidStatus.
   */
  inline const systems::InputPortDescriptor<double>&
  get_input_port_humanoid_status() const {
    return get_input_port(input_port_index_humanoid_status_);
  }

  /**
   * @return Port for the output: QPInput.
   */
  inline const systems::OutputPortDescriptor<double>& get_output_port_qp_input()
      const {
    return get_output_port(output_port_index_qp_input_);
  }

 private:
  const RigidBodyTree<double>& robot_;
  const double control_dt_{2e-3};

  param_parsers::RigidBodyTreeAliasGroups<double> alias_groups_;
  param_parsers::ParamSet paramset_;

  int input_port_index_humanoid_status_;
  int output_port_index_qp_input_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
