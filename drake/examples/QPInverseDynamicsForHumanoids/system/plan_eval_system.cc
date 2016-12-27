
#include <string>

#include "drake/examples/QPInverseDynamicsForHumanoids/system/plan_eval_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

using systems::Context;
using systems::SystemOutput;
using systems::State;
using systems::LeafSystemOutput;
using systems::AbstractValue;
using systems::AbstractState;
using systems::Value;

PlanEvalSystem::PlanEvalSystem(const RigidBodyTree<double>& robot) : robot_(robot) {
  input_port_index_humanoid_status_ = DeclareAbstractInputPort().get_index();
  output_port_index_qp_input_ = DeclareAbstractOutputPort().get_index();

  set_name("plan_eval");
}

void PlanEvalSystem::DoCalcOutput(const Context<double>& context,
    SystemOutput<double>* output) const {
  // Input:
  const HumanoidStatus* robot_status = EvalInputValue<HumanoidStatus>(
      context, input_port_index_humanoid_status_);

  // Output:
  lcmt_qp_input& msg = output->GetMutableData(output_port_index_qp_input_)
    ->GetMutableValue<lcmt_qp_input>();

  //const GenericHumanoidPlan& plan = context.get_abstract_state<GenericHumanoidPlan>(0);
  const HumanoidWalkingPlan& plan = context.get_abstract_state<HumanoidWalkingPlan>(0);
  QPInput qp_input = plan.CalcQPInput(*robot_status);
  EncodeQPInput(qp_input, &msg);
}

void PlanEvalSystem::DoCalcUnrestrictedUpdate(const Context<double>& context,
    State<double>* state) const {
  // Get the plan.
  AbstractValue& abs_val = state->get_mutable_abstract_state()->get_mutable_abstract_state(0);
  //GenericHumanoidPlan& plan = abs_val.GetMutableValue<GenericHumanoidPlan>();
  HumanoidWalkingPlan& plan = abs_val.GetMutableValue<HumanoidWalkingPlan>();

  // Get the state.
  const HumanoidStatus* robot_status = EvalInputValue<HumanoidStatus>(
      context, input_port_index_humanoid_status_);

  plan.DoStateTransition(*robot_status);
}

std::unique_ptr<SystemOutput<double>> PlanEvalSystem::AllocateOutput(
    const Context<double>& context) const {
  std::unique_ptr<LeafSystemOutput<double>> output(
      new LeafSystemOutput<double>);
  output->add_port(std::unique_ptr<AbstractValue>(
        new Value<lcmt_qp_input>(lcmt_qp_input())));
  return std::move(output);
}

void PlanEvalSystem::HandlePlan(const HumanoidStatus& planned, Context<double>* context) {
  // Make a walking plan
  HumanoidWalkingPlan plan(robot_);
  plan.HandleWalkingPlan(planned);

  std::vector<std::unique_ptr<AbstractValue>> abstract_vals(1);
  abstract_vals.front() = std::unique_ptr<AbstractValue>(new Value<HumanoidWalkingPlan>(plan));
  context->set_abstract_state(std::make_unique<AbstractState>(std::move(abstract_vals)));
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
