#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/plan_eval_base_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * This class extends PlanEvalBaseSystem. It generates QpInput to track
 * desired instantaneous position, velocity and acceleration in joint space.
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
                            const RigidBodyTree<double>& object,
                            const std::string& alias_groups_file_name,
                            const std::string& param_file_name, double dt);

  /**
   * Initializes the plan and gains. Must be called before execution.
   */
  void Initialize(const HumanoidStatus& status, systems::State<double>* state);

  /**
   * Initializes the plan and gains. Must be called before execution.
   */
  void Initialize(const HumanoidStatus& status, systems::Context<double>* context) {
    systems::State<double>* servo_state = context->get_mutable_state();
    Initialize(status, servo_state);
  }

  const systems::InputPortDescriptor<double>&
  get_input_port_plan() const {
    return get_input_port(input_port_index_plan_);
  }

  const systems::InputPortDescriptor<double>&
  get_input_port_object_state() const {
    return get_input_port(input_port_index_object_);
  }

  /**
   * Returns the output port for debugging information.
   */
  inline const systems::OutputPortDescriptor<double>&
  get_output_port_debug_info() const {
    return get_output_port(output_port_index_debug_info_);
  }

 private:
  int get_num_extended_abstract_states() const override { return 2; }

  void DoExtendedCalcUnrestrictedUpdate(
      const systems::Context<double>& context,
      systems::State<double>* state) const override;

  void DoExtendedCalcOutput(
      const systems::Context<double>& context,
      systems::SystemOutput<double>* output) const override;

  int input_port_index_plan_{};
  int input_port_index_object_{};
  int output_port_index_debug_info_{};

  int abs_state_index_plan_{};
  int abs_state_index_debug_{};

  mutable int64_t last_plan_timestamp_{0};

  const RigidBodyTree<double>& object_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
