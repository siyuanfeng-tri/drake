#include "drake/examples/QPInverseDynamicsForHumanoids/system/new_manipulator_plan_eval_system.h"

#include <vector>

#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/manipulator_plan.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller_common.h"
#include "drake/lcmt_plan_eval_debug_info.hpp"

#include "drake/lcmt_motion_plan.hpp"

#include "drake/manipulation/util/cartesian_trajectory_translator.h"
#include "drake/manipulation/util/dof_trajectory_translator.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

ManipulatorPlanEvalSystem::ManipulatorPlanEvalSystem(
    const RigidBodyTree<double>& robot,
    const RigidBodyTree<double>& object,
    const std::string& alias_groups_file_name,
    const std::string& param_file_name, double dt)
    : PlanEvalBaseSystem(robot, alias_groups_file_name, param_file_name, dt),
      object_(object) {
  DRAKE_DEMAND(get_robot().get_num_velocities() ==
               get_robot().get_num_positions());

  // Declare inputs.
  input_port_index_plan_ = DeclareAbstractInputPort().get_index();

  VectorX<double> obj_state = VectorX<double>::Zero(
      object_.get_num_positions() + object_.get_num_velocities());
  obj_state.head(object_.get_num_positions()) = object_.getZeroConfiguration();

  systems::BasicVector<double> obj_state0(obj_state);
  input_port_index_object_ = DeclareVectorInputPort(obj_state0).get_index();

  // Declare outputs.
  lcmt_plan_eval_debug_info debug_info;
  manipulation::DofKeyframeTranslator::InitializeMessage(7,
      &debug_info.dof_motion);
  debug_info.num_body_motions = 1;
  debug_info.body_motion_names.resize(debug_info.num_body_motions);
  debug_info.body_motions.resize(debug_info.num_body_motions);

  systems::Value<lcmt_plan_eval_debug_info> debug_msg(debug_info);
  output_port_index_debug_info_ =
      DeclareAbstractOutputPort(debug_msg).get_index();

  // Declare states.
  auto plan_as_value = systems::AbstractValue::Make<GenericPlan<double>>(
      ManipulatorPlan<double>(object));

  abs_state_index_plan_ =
      DeclareAbstractState(std::move(plan_as_value));
  abs_state_index_debug_ = DeclareAbstractState(
      systems::AbstractValue::Make<lcmt_plan_eval_debug_info>(debug_info));

  set_name("ManipulatorPlanEvalSystem");
}

void ManipulatorPlanEvalSystem::Initialize(const HumanoidStatus& current_status, systems::State<double>* state) {
  GenericPlan<double>& plan = get_mutable_abstract_value<GenericPlan<double>>(
      state, abs_state_index_plan_);
  plan.Initialize(current_status, get_paramset(), get_alias_groups());

  QpInput& qp_input = get_mutable_qp_input(state);

  VectorX<double> box_state = object_.getZeroConfiguration();
  plan.UpdateQpInput(current_status, get_paramset(), get_alias_groups(), &qp_input, &box_state);
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

  VectorX<double> obj_state =
      EvalEigenVectorInput(context, input_port_index_object_);

  // Gets the lcm message inputs.
  const lcmt_motion_plan* msg =
      EvalInputValue<lcmt_motion_plan>(
          context, input_port_index_plan_);

  // Gets the plan from abstract state.
  GenericPlan<double>& plan = get_mutable_abstract_value<GenericPlan<double>>(
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
    plan.HandlePlanMessage(*robot_status, get_paramset(),
        get_alias_groups(), raw_msg_bytes.data(), raw_msg_bytes.size());
  }

  // Runs controller.
  plan.ModifyPlan(*robot_status, get_paramset(), get_alias_groups());

  // Updates the QpInput in AbstractState.
  QpInput& qp_input = get_mutable_qp_input(state);

  // TODO
  plan.UpdateQpInput(*robot_status, get_paramset(), get_alias_groups(), &qp_input, &obj_state);

  // Generates debugging info.
  lcmt_plan_eval_debug_info& debug =
      get_mutable_abstract_value<lcmt_plan_eval_debug_info>(
          state, abs_state_index_debug_);

  plan.MakeDebugMessage(*robot_status, get_paramset(), get_alias_groups(), qp_input, &debug);
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
