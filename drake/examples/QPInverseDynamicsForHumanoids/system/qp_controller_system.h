#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * A discrete time system block for an inverse dynamics controller.
 */
class QPControllerSystem : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QPControllerSystem)

  /**
   * Constructor for the inverse dynamics controller.
   * @param robot Reference to a RigidBodyTree. An internal alias is saved,
   * so the lifespan of @p robot must be longer than this object.
   * @param dt Delta time between controls.
   */
  QPControllerSystem(const RigidBodyTree<double>& robot, double dt);

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  std::unique_ptr<systems::AbstractValue> AllocateOutputAbstract(
      const systems::OutputPortDescriptor<double>& descriptor) const override;

  std::unique_ptr<systems::AbstractState> AllocateAbstractState()
      const override;

  void DoCalcUnrestrictedUpdate(const systems::Context<double>& context,
                                systems::State<double>* state) const override;

  /**
   * @return Port for the input: HumanoidStatus.
   */
  inline const systems::InputPortDescriptor<double>&
  get_input_port_humanoid_status() const {
    return get_input_port(input_port_index_humanoid_status_);
  }

  /**
   * @return Port for the input: QpInput.
   */
  inline const systems::InputPortDescriptor<double>& get_input_port_qp_input()
      const {
    return get_input_port(input_port_index_qp_input_);
  }

  /**
   * @return Port for the output: QpOutput.
   */
  inline const systems::OutputPortDescriptor<double>&
  get_output_port_qp_output() const {
    return get_output_port(output_port_index_qp_output_);
  }

  /**
   * @return Port for the output: lcmt_inverse_dynamics_debug_info.
   */
  inline const systems::OutputPortDescriptor<double>&
  get_output_port_debug_info() const {
    return get_output_port(output_port_index_debug_info_);
  }

 private:
  QpOutput& get_mutable_qp_output(systems::State<double>* state) const {
    return state->get_mutable_abstract_state()
        ->get_mutable_abstract_state(kAbsStateIdxQpOutput)
        .GetMutableValue<QpOutput>();
  }

  const RigidBodyTree<double>& robot_;
  const double kControlDt;
  const int kAbsStateIdxQpOutput;
  const int kAbsStateIdxDebug;

  // TODO(siyuan.feng): This is a bad temporary hack to the const constraint for
  // CalcOutput. It is because qp controller needs to allocate mutable workspace
  // (MathematicalProgram, temporary matrices for doing math, etc),
  // and I want to avoid allocating these repeatedly.
  // This should be taken care of with the new system2 cache.
  mutable QPController qp_controller_;

  int input_port_index_humanoid_status_{0};
  int input_port_index_qp_input_{0};
  int output_port_index_qp_output_{0};
  int output_port_index_debug_info_{0};
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
