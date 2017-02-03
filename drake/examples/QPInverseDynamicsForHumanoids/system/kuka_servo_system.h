#pragma once

#include <memory>
#include <string>
#include <utility>

#include "drake/examples/QPInverseDynamicsForHumanoids/system/discrete_time_plan_eval_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * This class extends DiscreteTimePlanEvalSystem. It generates QpInput to track
 * a desired trajectory.
 */
class KukaServoSystem : public DiscreteTimePlanEvalSystem {
 public:
  KukaServoSystem(const RigidBodyTree<double>& robot,
                     const std::string& alias_groups_file_name,
                     const std::string& param_file_name, double dt);

  void DoCalcUnrestrictedUpdate(const systems::Context<double>& context,
                                systems::State<double>* state) const override;

  std::unique_ptr<systems::AbstractState> AllocateAbstractState()
      const override;

  void Initialize(systems::State<double>* state);

  void Initialize(systems::Context<double>* context) {
    systems::State<double>* servo_state =
        context->get_mutable_state();
    Initialize(servo_state);
  }

  /**
   * @return Port for the input: HumanoidStatus.
   */
  inline const systems::InputPortDescriptor<double>&
  get_input_port_desired_state() const {
    return get_input_port(input_port_index_desired_state_);
  }

 private:
  int input_port_index_desired_state_{0};
  mutable VectorX<double> last_v_d_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
