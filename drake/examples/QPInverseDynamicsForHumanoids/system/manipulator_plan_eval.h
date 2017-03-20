#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/plan_eval_base_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * This class extends PlanEvalBaseSystem. It generates QpInput to track
 * desired instantaneous position, velocity and acceleration in joint space.
 */
class ManipulatorPlanEval : public PlanEvalBaseSystem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ManipulatorPlanEval)

  /**
   * Constructor.
   * @param robot Reference to a RigidBodyTree, whose life span must be longer
   * than this instance.
   * @param alias_groups_file_name Path to the alias groups file that describes
   * the robot's topology for the controller.
   * @param param_file_name Path to the config file for the controller.
   * @param dt Time step
   */
  ManipulatorPlanEval(const RigidBodyTree<double>& robot,
                            const std::string& alias_groups_file_name,
                            const std::string& param_file_name, double dt);

  /**
   * Returns the input port for desired position and velocity.
   */
  inline const systems::InputPortDescriptor<double>&
  get_input_port_move_joints_command() const {
    return get_input_port(input_port_index_move_joints_command_);
  }

  /**
   * Returns the input port for desired acceleration.
   */
  inline const systems::InputPortDescriptor<double>&
  get_input_port_move_end_effector_command() const {
    return get_input_port(input_port_index_move_ee_command_);
  }

  /**
   * Returns the output port for debugging information.
   */
  inline const systems::OutputPortDescriptor<double>&
  get_output_port_debug_info() const {
    return get_output_port(output_port_index_debug_info_);
  }

 private:
  void DoInitializePlan(const HumanoidStatus& current_status,
                        systems::State<double>* state);

  int get_num_extended_abstract_states() const { return 2; }

  void DoExtendedCalcUnrestrictedUpdate(const systems::Context<double>& context,
                                        systems::State<double>* state) const;

  void DoExtendedCalcOutput(const systems::Context<double>& context,
                            systems::SystemOutput<double>* output) const;

  std::vector<std::unique_ptr<systems::AbstractValue>>
  ExtendedAllocateAbstractState() const;

  std::unique_ptr<systems::AbstractValue> ExtendedAllocateOutputAbstract(
      const systems::OutputPortDescriptor<double>& descriptor) const;

  template <typename PlanType>
  void MakeNewPlan(const HumanoidStatus& robot_status, const std::vector<uint8_t>& raw_msg_bytes, systems::State<double>* state) const;

  int input_port_index_move_joints_command_{};
  int input_port_index_move_ee_command_{};
  int output_port_index_debug_info_{};

  const int abs_state_index_plan_{};
  const int abs_state_index_debug_{};

  mutable int64_t last_move_ee_timestamp_{0};
  mutable int64_t last_move_joints_timestamp_{0};
};








}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
