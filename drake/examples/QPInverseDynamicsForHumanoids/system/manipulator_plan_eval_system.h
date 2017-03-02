#pragma once

#include <memory>
#include <string>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/plan_eval_base_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * This class extends PlanEvalBaseSystem. It generates QpInput to track
 * desired instantaneous position, velocity and acceleration.
 */
class ManipulatorPlanEvalSystem : public PlanEvalBaseSystem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ManipulatorPlanEvalSystem)

  /**
   * Constructor.
   * @param robot Reference to a RigidBodyTree, whose life span must be longer
   * than this instance.
   * @param alias_groups_file_name Path to the alias groups file that describes
   * the robot's topology for the controller.
   * @param param_file_name Path to the config file for the controller.
   * @param dt Time step
   */
  ManipulatorPlanEvalSystem(const RigidBodyTree<double>& robot,
                            const std::string& alias_groups_file_name,
                            const std::string& param_file_name, double dt);

  void DoCalcUnrestrictedUpdate(const systems::Context<double>& context,
                                systems::State<double>* state) const override;

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  std::unique_ptr<systems::AbstractState> AllocateAbstractState()
      const override;

  std::unique_ptr<systems::AbstractValue> AllocateOutputAbstract(
      const systems::OutputPortDescriptor<double>& descriptor) const override;

  /**
   * Initializes the internal states.
   */
  void Initialize(systems::State<double>* state);

  /**
   * Initializes the internal states.
   */
  void Initialize(systems::Context<double>* context) {
    systems::State<double>* servo_state = context->get_mutable_state();
    Initialize(servo_state);
  }

  /**
   * Returns the input port for desired position and velocity.
   */
  inline const systems::InputPortDescriptor<double>&
  get_input_port_desired_state() const {
    return get_input_port(input_port_index_desired_state_);
  }

  /**
   * Returns the input port for desired acceleration.
   */
  inline const systems::InputPortDescriptor<double>&
  get_input_port_desired_acceleration() const {
    return get_input_port(input_port_index_desired_acceleration_);
  }

  /**
   * Returns the output port for debugging information.
   */
  inline const systems::OutputPortDescriptor<double>&
  get_output_port_debug_info() const {
    return get_output_port(output_port_index_debug_info_);
  }

 private:
  int input_port_index_desired_state_{0};
  int input_port_index_desired_acceleration_{0};
  int output_port_index_debug_info_{0};

  const int kAbsStateIdxDebug;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
