#include "drake/examples/QPInverseDynamicsForHumanoids/system/manipulator_plan_eval.h"

#include <vector>

#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller_common.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/manipulator_move_joints_plan.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/manipulator_move_end_effector_plan.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/clone_only_value.h"

#include "robotlocomotion/robot_plan_t.hpp"
#include "drake/lcmt_plan_eval_debug_info.hpp"
#include "drake/lcmt_manipulator_plan_move_end_effector.hpp"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

template <typename CloneOnlyType>
CloneOnlyType* get_mutable_plan(systems::State<double>* state, int index) {
  systems::CloneOnlyValue<CloneOnlyType>& plan_ptr =
      state->get_mutable_abstract_state()->get_mutable_value(index).
          GetMutableValue<systems::CloneOnlyValue<CloneOnlyType>>();
  return plan_ptr.get_value();
}

ManipulatorPlanEval::ManipulatorPlanEval(
    const RigidBodyTree<double>& robot,
    const std::string& alias_groups_file_name,
    const std::string& param_file_name, double dt)
    : PlanEvalBaseSystem(robot, alias_groups_file_name, param_file_name, dt),
      abs_state_index_plan_(0),
      abs_state_index_debug_(1) {
  DRAKE_DEMAND(get_robot().get_num_velocities() ==
               get_robot().get_num_positions());

  input_port_index_move_joints_command_ =
      DeclareAbstractInputPort().get_index();
  input_port_index_move_ee_command_ =
      DeclareAbstractInputPort().get_index();

  output_port_index_debug_info_ = DeclareAbstractOutputPort().get_index();
  set_name("ManipulatorPlanEval");
}

void ManipulatorPlanEval::DoExtendedCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  // Copies additional debugging info from abstract state to output.
  lcmt_plan_eval_debug_info& debug =
      output->GetMutableData(output_port_index_debug_info_)
          ->GetMutableValue<lcmt_plan_eval_debug_info>();
  debug = context.get_abstract_state<lcmt_plan_eval_debug_info>(
      abs_state_index_debug_);
}

void ManipulatorPlanEval::DoInitializePlan(const HumanoidStatus& current_status, systems::State<double>* state) {
  GenericPlan<double>* plan = get_mutable_plan<GenericPlan<double>>(state, abs_state_index_plan_);
  plan->Initialize(current_status, get_paramset(), get_alias_groups());

  QpInput& qp_input = get_mutable_qp_input(state);
  plan->UpdateQpInput(current_status, get_paramset(), get_alias_groups(), &qp_input);
}

template <typename PlanType>
void ManipulatorPlanEval::MakeNewPlan(
    const HumanoidStatus& robot_status,
    const std::vector<uint8_t>& raw_msg_bytes,
    systems::State<double>* state) const {
  systems::CloneOnlyValue<GenericPlan<double>> new_plan(
      std::make_unique<PlanType>());
  systems::CloneOnlyValue<GenericPlan<double>>& plan_ptr =
    state->get_mutable_abstract_state()->
    get_mutable_value(abs_state_index_plan_).
    GetMutableValue<systems::CloneOnlyValue<GenericPlan<double>>>();
  plan_ptr = new_plan;
  std::cout << "Changed plan\n";

  plan_ptr.get_value()->Initialize(robot_status, get_paramset(), get_alias_groups());
  plan_ptr.get_value()->HandlePlanMessage(robot_status, get_paramset(), get_alias_groups(), raw_msg_bytes.data(), raw_msg_bytes.size());
}

void ManipulatorPlanEval::DoExtendedCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  // Gets the robot state from input.
  const HumanoidStatus* robot_status = EvalInputValue<HumanoidStatus>(
      context, get_input_port_humanoid_status().get_index());

  // Gets the lcm message inputs.
  const lcmt_manipulator_plan_move_end_effector* move_ee_msg =
      EvalInputValue<lcmt_manipulator_plan_move_end_effector>(
          context, input_port_index_move_ee_command_);

  const robotlocomotion::robot_plan_t* move_joints_msg =
      EvalInputValue<robotlocomotion::robot_plan_t>(
          context, input_port_index_move_joints_command_);

  bool new_move_ee_flag = false;
  bool new_move_joint_flag = false;

  if (last_move_ee_timestamp_ != move_ee_msg->timestamp) {
    new_move_ee_flag = true;
    last_move_ee_timestamp_ = move_ee_msg->timestamp;
  }

  if (last_move_joints_timestamp_ != move_joints_msg->utime) {
    new_move_joint_flag = true;
    last_move_joints_timestamp_ = move_joints_msg->utime;
  }

  if (new_move_ee_flag && new_move_joint_flag) {
    DRAKE_ABORT_MSG("can't handle move commands at the same time.");
  }

  if (new_move_ee_flag) {
    std::vector<uint8_t> raw_msg_bytes;
    raw_msg_bytes.resize(move_ee_msg->getEncodedSize());
    move_ee_msg->encode(raw_msg_bytes.data(), 0, raw_msg_bytes.size());
    MakeNewPlan<ManipulatorMoveEndEffectorPlan<double>>(*robot_status, raw_msg_bytes, state);
  }

  if (new_move_joint_flag) {
    std::vector<uint8_t> raw_msg_bytes;
    raw_msg_bytes.resize(move_joints_msg->getEncodedSize());
    move_joints_msg->encode(raw_msg_bytes.data(), 0, raw_msg_bytes.size());
    MakeNewPlan<ManipulatorMoveJointsPlan<double>>(*robot_status, raw_msg_bytes, state);
  }

  // Gets the plan from abstract state.
  GenericPlan<double>* plan = get_mutable_plan<GenericPlan<double>>(state, abs_state_index_plan_);

  // Runs controller.
  plan->ExecutePlan(*robot_status, get_paramset(), get_alias_groups());

  // Updates the QpInput in AbstractState.
  QpInput& qp_input = get_mutable_qp_input(state);
  plan->UpdateQpInput(*robot_status, get_paramset(), get_alias_groups(), &qp_input);
}

std::vector<std::unique_ptr<systems::AbstractValue>>
ManipulatorPlanEval::ExtendedAllocateAbstractState() const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals(
      get_num_extended_abstract_states());

  abstract_vals[abs_state_index_plan_] =
      systems::AbstractValue::Make<systems::CloneOnlyValue<GenericPlan<double>>>(
          systems::CloneOnlyValue<GenericPlan<double>>(
              std::make_unique<ManipulatorMoveJointsPlan<double>>()));

  abstract_vals[abs_state_index_debug_] =
      std::unique_ptr<systems::AbstractValue>(
          new systems::Value<lcmt_plan_eval_debug_info>(
              lcmt_plan_eval_debug_info()));

  return abstract_vals;
}

std::unique_ptr<systems::AbstractValue>
ManipulatorPlanEval::ExtendedAllocateOutputAbstract(
    const systems::OutputPortDescriptor<double>& descriptor) const {
  if (descriptor.get_index() == output_port_index_debug_info_) {
    return systems::AbstractValue::Make<lcmt_plan_eval_debug_info>(
        lcmt_plan_eval_debug_info());
  }
  std::string msg = "ManipulatorPlanEval does not have outputport " +
                    std::to_string(descriptor.get_index());
  DRAKE_ABORT_MSG(msg.c_str());
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
