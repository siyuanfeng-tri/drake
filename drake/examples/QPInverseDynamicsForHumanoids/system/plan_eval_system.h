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
 * Input: HumanoidStatus
 * Output: QPInput
 */
class PlanEvalSystem : public systems::LeafSystem<double> {
 public:
  explicit PlanEvalSystem(const RigidBodyTree<double>& robot);

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  // This is used to trigger DoCalcUnrestrictedUpdate every control_dt_ when
  // this is wired up with a Simulator.
  void DoCalcNextUpdateTime(
      const systems::Context<double>& context,
      systems::UpdateActions<double>* actions) const override;

  void DoCalcUnrestrictedUpdate(const systems::Context<double>& context,
                                systems::State<double>* state) const override;

  std::unique_ptr<systems::SystemOutput<double>> AllocateOutput(
      const systems::Context<double>& context) const override;

  /**
   * Set the set point for tracking.
   * @param robot_status, desired robot state
   */
  void SetDesired(const VectorX<double>& q, systems::State<double>* state);

  std::unique_ptr<systems::AbstractState> AllocateAbstractState()
      const override;

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
