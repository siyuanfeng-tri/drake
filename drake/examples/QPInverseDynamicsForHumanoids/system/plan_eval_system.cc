
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

PlanEvalSystem::PlanEvalSystem(const RigidBodyTree<double>& robot) : robot_(robot), qp_input_(QPInput(robot)) {
    //: robot_(robot), manip_plan_eval_(HumanoidManipPlan(robot)), qp_input_(QPInput(robot)) {
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

  const HumanoidWalkingPlan& plan = context.get_abstract_state<HumanoidWalkingPlan>(0);
  plan.UpdateQPInput(*robot_status, &qp_input_);
  EncodeQPInput(qp_input_, &msg);
}

void PlanEvalSystem::DoCalcUnrestrictedUpdate(const Context<double>& context,
    State<double>* state) const {
  // Get the plan.
  HumanoidWalkingPlan& plan = state->get_mutable_abstract_state()->get_mutable_abstract_state(0).GetMutableValue<HumanoidWalkingPlan>();

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

void PlanEvalSystem::SetDesired(const HumanoidStatus& robot_status, Context<double>* context) {
  // Make a walking plan
  std::vector<std::unique_ptr<AbstractValue>> abstract_vals(1);
  std::unique_ptr<AbstractValue> plan(new Value<HumanoidWalkingPlan>(HumanoidWalkingPlan(robot_)));

  plan->GetMutableValue<HumanoidWalkingPlan>().HandleWalkingPlan(robot_status);

  abstract_vals.front() = std::move(plan);
  context->set_abstract_state(std::make_unique<AbstractState>(std::move(abstract_vals)));

  //manip_plan_eval_.HandleWalkingPlan(robot_status, &qp_input_);
  //manip_plan_eval_.HandleManipPlan(robot_status, &qp_input_);
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
