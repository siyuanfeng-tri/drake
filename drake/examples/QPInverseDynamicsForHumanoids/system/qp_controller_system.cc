#include "drake/examples/QPInverseDynamicsForHumanoids/system/qp_controller_system.h"

#include <memory>
#include <utility>
#include <vector>

#include "drake/lcmt_inverse_dynamics_debug_info.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

QPControllerSystem::QPControllerSystem(const RigidBodyTree<double>& robot,
                                       double dt)
    : robot_(robot), kControlDt(dt), kAbsStateIdxQpOutput(0), kAbsStateIdxDebug(1) {
  input_port_index_humanoid_status_ = DeclareAbstractInputPort().get_index();
  input_port_index_qp_input_ = DeclareAbstractInputPort().get_index();
  output_port_index_qp_output_ = DeclareAbstractOutputPort().get_index();
  output_port_index_debug_info_ = DeclareAbstractOutputPort().get_index();

  set_name("QPControllerSystem");
  DeclarePeriodicUnrestrictedUpdate(kControlDt, 0);
}

void QPControllerSystem::DoCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  QpOutput& qp_output = output->GetMutableData(output_port_index_qp_output_)
                            ->GetMutableValue<QpOutput>();
  qp_output =
      context.get_abstract_state<QpOutput>(kAbsStateIdxQpOutput);

  lcmt_inverse_dynamics_debug_info& debug = output->GetMutableData(output_port_index_debug_info_)
      ->GetMutableValue<lcmt_inverse_dynamics_debug_info>();

  debug =
      context.get_abstract_state<lcmt_inverse_dynamics_debug_info>(kAbsStateIdxDebug);
}

void QPControllerSystem::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  // Inputs:
  const HumanoidStatus* rs = EvalInputValue<HumanoidStatus>(
      context, input_port_index_humanoid_status_);

  const QpInput* qp_input =
      EvalInputValue<QpInput>(context, input_port_index_qp_input_);

  // Calls the controller.
  QpOutput& qp_output = get_mutable_qp_output(state);

  if (qp_controller_.Control(*rs, *qp_input, &qp_output) < 0) {
    std::stringstream err;
    err << rs->position().transpose() << "\n";
    err << rs->velocity().transpose() << "\n";
    err << *qp_input << std::endl;
    throw std::runtime_error("QPControllerSystem: QP cannot solve\n" +
                             err.str());
  }

  // Generates debugging info.
  lcmt_inverse_dynamics_debug_info& debug = state->get_mutable_abstract_state()->get_mutable_abstract_state(kAbsStateIdxDebug).GetMutableValue<lcmt_inverse_dynamics_debug_info>();
  debug.timestamp = context.get_time() * 1e6;
  debug.num_dof = robot_.get_num_velocities();
  debug.dof_names = GetDofNames(robot_);
  debug.desired_vd.resize(debug.num_dof);
  debug.solved_vd.resize(debug.num_dof);
  debug.solved_torque.resize(debug.num_dof);
  for (int i = 0; i < debug.num_dof; ++i) {
    debug.desired_vd[i] = qp_input->desired_dof_motions().value(i);
    debug.solved_vd[i] = qp_output.vd()[i];
    debug.solved_torque[i] = qp_output.dof_torques()[i];
  }
}

std::unique_ptr<systems::AbstractState>
QPControllerSystem::AllocateAbstractState() const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals(2);
  abstract_vals[kAbsStateIdxQpOutput] =
      std::unique_ptr<systems::AbstractValue>(
          new systems::Value<QpOutput>(QpOutput(GetDofNames(robot_))));
  abstract_vals[kAbsStateIdxDebug] =
      std::unique_ptr<systems::AbstractValue>(
          new systems::Value<lcmt_inverse_dynamics_debug_info>(lcmt_inverse_dynamics_debug_info()));
  return std::make_unique<systems::AbstractState>(std::move(abstract_vals));
}

std::unique_ptr<systems::AbstractValue>
QPControllerSystem::AllocateOutputAbstract(
    const systems::OutputPortDescriptor<double>& descriptor) const {
  if (descriptor.get_index() == output_port_index_qp_output_) {
    return systems::AbstractValue::Make<QpOutput>(QpOutput(GetDofNames(robot_)));
  } else if (descriptor.get_index() == output_port_index_debug_info_) {
    return systems::AbstractValue::Make<lcmt_inverse_dynamics_debug_info>(lcmt_inverse_dynamics_debug_info());
  } else {
    DRAKE_DEMAND(false);
    return nullptr;
  }
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
