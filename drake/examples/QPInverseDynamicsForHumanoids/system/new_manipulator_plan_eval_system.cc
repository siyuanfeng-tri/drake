#include "drake/examples/QPInverseDynamicsForHumanoids/system/new_manipulator_plan_eval_system.h"

#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/manipulator_plan.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller_common.h"
#include "drake/lcmt_plan_eval_debug_info.hpp"

#include "drake/lcmt_motion_plan.hpp"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

ManipulatorPlanEvalSystem::ManipulatorPlanEvalSystem(
    const RigidBodyTree<double>& robot,
    const std::string& alias_groups_file_name,
    const std::string& param_file_name, double dt)
    : PlanEvalBaseSystem(robot, alias_groups_file_name, param_file_name, dt) {
  DRAKE_DEMAND(get_robot().get_num_velocities() ==
               get_robot().get_num_positions());

  // Declare inputs.
  input_port_index_plan_ = DeclareAbstractInputPort().get_index();

  // Declare outputs.
  lcmt_plan_eval_debug_info debug_info;
  systems::Value<lcmt_plan_eval_debug_info> debug_msg(debug_info);
  output_port_index_debug_info_ =
      DeclareAbstractOutputPort(debug_msg).get_index();

  // Declare states.
  copyable_unique_ptr<GenericPlan<double>> plan(new ManipulatorPlan<double>());
  auto plan_as_value = systems::AbstractValue::Make<copyable_unique_ptr<GenericPlan<double>>>(
      plan);

  abs_state_index_plan_ =
      DeclareAbstractState(std::move(plan_as_value));
  abs_state_index_debug_ = DeclareAbstractState(
      systems::AbstractValue::Make<lcmt_plan_eval_debug_info>(debug_info));

  set_name("ManipulatorPlanEvalSystem");
}

void ManipulatorPlanEvalSystem::Initialize(const HumanoidStatus& current_status, systems::State<double>* state) {
  copyable_unique_ptr<GenericPlan<double>>& plan_ptr =
      get_mutable_abstract_value<copyable_unique_ptr<GenericPlan<double>>>(
          state, abs_state_index_plan_);

  plan_ptr->Initialize(current_status, get_paramset(), get_alias_groups());

  QpInput& qp_input = get_mutable_qp_input(state);
  const std::vector<std::string> empty;
  plan_ptr->UpdateQpInput(current_status, get_paramset(), get_alias_groups(), &qp_input);
}

void ManipulatorPlanEvalSystem::DoExtendedCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  // Copies additional debugging info from abstract state to output.
  lcmt_plan_eval_debug_info& debug =
      output->GetMutableData(output_port_index_debug_info_)
          ->GetMutableValue<lcmt_plan_eval_debug_info>();
  debug = context.get_abstract_state<lcmt_plan_eval_debug_info>(
      abs_state_index_debug_);
}

void ManipulatorPlanEvalSystem::DoExtendedCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  // Gets the robot state from input.
  const HumanoidStatus* robot_status = EvalInputValue<HumanoidStatus>(
      context, get_input_port_humanoid_status().get_index());

  // Gets the lcm message inputs.
  const lcmt_motion_plan* msg =
      EvalInputValue<lcmt_motion_plan>(
          context, input_port_index_plan_);

  // Gets the plan from abstract state.
  copyable_unique_ptr<GenericPlan<double>>& plan_ptr =
      get_mutable_abstract_value<copyable_unique_ptr<GenericPlan<double>>>(
          state, abs_state_index_plan_);

  bool new_plan_flag = false;
  if (last_plan_timestamp_ != msg->timestamp) {
    new_plan_flag = true;
    last_plan_timestamp_ = msg->timestamp;
  }

  if (new_plan_flag) {
    std::vector<uint8_t> raw_msg_bytes;
    raw_msg_bytes.resize(msg->getEncodedSize());
    msg->encode(raw_msg_bytes.data(), 0, raw_msg_bytes.size());
    plan_ptr->HandlePlanMessage(*robot_status, get_paramset(),
        get_alias_groups(), raw_msg_bytes.data(), raw_msg_bytes.size());
  }

  // Runs controller.
  plan_ptr->ModifyPlan(*robot_status, get_paramset(), get_alias_groups());

  // Updates the QpInput in AbstractState.
  QpInput& qp_input = get_mutable_qp_input(state);
  plan_ptr->UpdateQpInput(*robot_status, get_paramset(), get_alias_groups(), &qp_input);

  /*
  // Generates debugging info.
  lcmt_plan_eval_debug_info& debug =
      get_mutable_abstract_value<lcmt_plan_eval_debug_info>(
          state, abs_state_index_debug_);

  debug.timestamp = static_cast<int64_t>(context.get_time() * 1e6);
  debug.num_dof = dim;
  debug.dof_names.resize(dim);
  debug.nominal_q.resize(dim);
  debug.nominal_v.resize(dim);
  debug.nominal_vd.resize(dim);

  for (int i = 0; i < dim; i++) {
    debug.dof_names[i] = get_robot().get_position_name(i);
    debug.nominal_q[i] = plan.desired_position()[i];
    debug.nominal_v[i] = plan.desired_velocity()[i];
    debug.nominal_vd[i] = plan.desired_acceleration()[i];
  }
  */
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
