#pragma once

#include <memory>
#include <string>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * A translator class from state vector to HumanoidStatus.
 */
class StateToHumanoidStatus : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StateToHumanoidStatus)

  /**
   * Constructor.
   * @param robot Reference to a RigidBodyTree. An internal alias is saved,
   * so the lifespan of @p robot must be longer than this object.
   * @param alias_group_path Path to the alias groups file. Used by the
   * controller to understand to topology of the robot.
   */
  StateToHumanoidStatus(const RigidBodyTree<double>& robot,
                        const std::string& alias_group_path);

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  std::unique_ptr<systems::SystemOutput<double>> AllocateOutput(
      const systems::Context<double>& context) const override;

  /**
   * @return Port for the input: lcm message bot_core::robot_state_t
   */
  inline const systems::InputPortDescriptor<double>& get_input_port_state()
      const {
    return get_input_port(input_port_index_state_);
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
  const std::string alias_group_path_;

  int input_port_index_state_{0};
  int output_port_index_humanoid_status_{0};
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
