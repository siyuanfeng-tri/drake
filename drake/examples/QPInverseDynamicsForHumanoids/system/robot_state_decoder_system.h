#pragma once

#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * A translator from bot_core::robot_state_t to HumanoidStatus
 *
 * Input: lcm message bot_core::robot_state_t
 * Output: HumanoidStatus
 */
class RobotStateDecoderSystem : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RobotStateDecoderSystem)

  /**
   * Constructor.
   * @param robot Reference to a RigidBodyTree. An internal alias is saved,
   * so the lifespan of @p robot must be longer than this object.
   * @param alias_group_path Path to the alias groups file. Used by the
   * controller to understand to topology of the robot.
   */
  RobotStateDecoderSystem(const RigidBodyTree<double>& robot,
                          const std::string& alias_group_path);

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  std::unique_ptr<systems::AbstractValue> AllocateOutputAbstract(
      const systems::OutputPortDescriptor<double>& descriptor) const override;

  /**
   * @return Port for the input: lcm message bot_core::robot_state_t
   */
  inline const systems::InputPortDescriptor<double>&
  get_input_port_robot_state_msg() const {
    return get_input_port(input_port_index_lcm_msg_);
  }

  /**
   * @return Port for the output: HumanoidStatus.
   */
  inline const systems::OutputPortDescriptor<double>&
  get_output_port_humanoid_status() const {
    return get_output_port(output_port_index_humanoid_status_);
  }

 private:
  const RigidBodyTree<double>& robot_;
  std::string alias_group_path_;

  int input_port_index_lcm_msg_{0};
  int output_port_index_humanoid_status_{0};
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
