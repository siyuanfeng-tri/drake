#include <iostream>
#include <memory>
#include <utility>

#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/qp_controller_system.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

QPControllerSystem::QPControllerSystem(const RigidBodyTree<double>& robot)
    : robot_(robot) {
  input_port_index_humanoid_status_ = DeclareAbstractInputPort().get_index();
  input_port_index_qp_input_ = DeclareAbstractInputPort().get_index();
  output_port_index_qp_input_ = DeclareAbstractOutputPort().get_index();

  DRAKE_ASSERT(this->get_num_input_ports() == 2);
  DRAKE_ASSERT(this->get_num_output_ports() == 1);

  set_name("qp_controller");
}

void QPControllerSystem::DoCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  // Inputs:
  const HumanoidStatus* rs = EvalInputValue<HumanoidStatus>(
      context, input_port_index_humanoid_status_);

  const lcmt_qp_input* qp_input_msg =
      EvalInputValue<lcmt_qp_input>(context, input_port_index_qp_input_);

  QPInput qp_input;
  DecodeQPInput(robot_, *qp_input_msg, &qp_input);

  // Output:
  QPOutput& qp_output = output->GetMutableData(output_port_index_qp_input_)
                            ->GetMutableValue<QPOutput>();

  if (qp_controller_.Control(*rs, qp_input, &qp_output) < 0) {
    std::cout << rs->position().transpose() << std::endl;
    std::cout << rs->velocity().transpose() << std::endl;
    std::cout << qp_input << std::endl;
    throw std::runtime_error("System2QP: QP cannot solve\n");
  }
}

std::unique_ptr<systems::SystemOutput<double>>
QPControllerSystem::AllocateOutput(
    const systems::Context<double>& context) const {
  std::unique_ptr<systems::LeafSystemOutput<double>> output(
      new systems::LeafSystemOutput<double>);
  output->add_port(std::unique_ptr<systems::AbstractValue>(
      new systems::Value<QPOutput>(QPOutput(GetDoFNames(robot_)))));
  return std::move(output);
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
