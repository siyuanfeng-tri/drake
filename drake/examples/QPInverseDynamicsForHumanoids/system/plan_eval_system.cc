
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
  /*
  // Input:
  const HumanoidStatus* robot_status = EvalInputValue<HumanoidStatus>(
      context, input_port_index_humanoid_status_);

  // Output:
  lcmt_qp_input& msg = output->GetMutableData(output_port_index_qp_input_)
    ->GetMutableValue<lcmt_qp_input>();

  //const GenericHumanoidPlan& plan = context.get_abstract_state<GenericHumanoidPlan>(0);
  const HumanoidWalkingPlan& plan = context.get_abstract_state<HumanoidWalkingPlan>(0);
  QPInput qp_input = plan.CalcQPInput(*robot_status);
  */

  lcmt_qp_input& msg = output->GetMutableData(output_port_index_qp_input_)
    ->GetMutableValue<lcmt_qp_input>();
  const QPInput& qp_input = context.get_abstract_state<QPInput>(1);
  EncodeQPInput(qp_input, &msg);
}

void PlanEvalSystem::DoCalcNextUpdateTime(
    const systems::Context<double>& context,
    systems::UpdateActions<double>* actions) const {
  actions->time = context.get_time() + time_step_;
}

void PlanEvalSystem::DoCalcUnrestrictedUpdate(const Context<double>& context,
    State<double>* state) const {
  AbstractState* abs_state = state->get_mutable_abstract_state();
  DRAKE_DEMAND(abs_state->size() == 2);

  // Get the plan.
  HumanoidWalkingPlan& plan = abs_state->get_mutable_abstract_state(0).GetMutableValue<HumanoidWalkingPlan>();

  // Get QPInput
  QPInput& qp_input = abs_state->get_mutable_abstract_state(1).GetMutableValue<QPInput>();

  // Get the state.
  const HumanoidStatus* robot_status = EvalInputValue<HumanoidStatus>(
      context, input_port_index_humanoid_status_);

  std::cout << context.get_time() << ", " << robot_status->time() << std::endl;

  plan.DoStateTransition(*robot_status);
  qp_input = plan.CalcQPInput(*robot_status);
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

  std::vector<std::unique_ptr<AbstractValue>> abstract_vals;
  abstract_vals.reserve(2);
  abstract_vals.push_back(std::unique_ptr<AbstractValue>(new Value<HumanoidWalkingPlan>(plan)));
  abstract_vals.push_back(std::unique_ptr<AbstractValue>(new Value<QPInput>(QPInput(robot_))));
  context->set_abstract_state(std::make_unique<AbstractState>(std::move(abstract_vals)));
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
