#include "drake/examples/QPInverseDynamicsForHumanoids/system/manipulator_plan_eval_system.h"

#include <vector>

#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller_common.h"
#include "drake/lcmt_plan_eval_debug_info.hpp"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

ManipulatorPlanEvalSystem::ManipulatorPlanEvalSystem(
    const RigidBodyTree<double>& robot,
    const std::string& alias_groups_file_name,
    const std::string& param_file_name, double dt)
    : PlanEvalBaseSystem(robot, alias_groups_file_name, param_file_name, dt),
      kAbsStateIdxDebug(2) {
  DRAKE_DEMAND(get_robot().get_num_velocities() ==
               get_robot().get_num_positions());
  const int kStateDim =
      get_robot().get_num_positions() + get_robot().get_num_velocities();
  const int kAccDim = get_robot().get_num_velocities();

  input_port_index_desired_state_ =
      DeclareInputPort(systems::kVectorValued, kStateDim).get_index();
  input_port_index_desired_acceleration_ =
      DeclareInputPort(systems::kVectorValued, kAccDim).get_index();

  output_port_index_debug_info_ = DeclareAbstractOutputPort().get_index();
  set_name("ManipulatorPlanEvalSystem");
}

void ManipulatorPlanEvalSystem::Initialize(systems::State<double>* state) {
  VectorSetpoint<double>& plan =
      get_mutable_plan<VectorSetpoint<double>>(state);
  plan = VectorSetpoint<double>(get_robot().get_num_velocities());
  get_paramset().LookupDesiredDofMotionGains(&(plan.mutable_Kp()),
                                             &(plan.mutable_Kd()));

  QpInput& qp_input = get_mutable_qp_input(state);
  qp_input = get_paramset().MakeQpInput({}, /* contacts */
                                        {}, /* tracked bodies */
                                        get_alias_groups());
}

void ManipulatorPlanEvalSystem::DoCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  // Does the normal plan eval step.
  PlanEvalBaseSystem::DoCalcOutput(context, output);

  // Copies additional debugging info from abstract state to output.
  lcmt_plan_eval_debug_info& debug =
      output->GetMutableData(output_port_index_debug_info_)
          ->GetMutableValue<lcmt_plan_eval_debug_info>();
  debug =
      context.get_abstract_state<lcmt_plan_eval_debug_info>(kAbsStateIdxDebug);
}

void ManipulatorPlanEvalSystem::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  // Gets the plan from abstract state.
  VectorSetpoint<double>& plan =
      get_mutable_plan<VectorSetpoint<double>>(state);

  // Gets the robot state from input.
  const HumanoidStatus* robot_status = EvalInputValue<HumanoidStatus>(
      context, get_input_port_index_humanoid_status());

  // Gets the desired position and velocity.
  const systems::BasicVector<double>* state_d = EvalVectorInput(
      context, input_port_index_desired_state_);

  // Gets the desired acceleration.
  const systems::BasicVector<double>* acc_d = EvalVectorInput(
      context, input_port_index_desired_acceleration_);

  QpInput& qp_input = get_mutable_qp_input(state);
  qp_input = get_paramset().MakeQpInput({}, /* contacts */
                                        {}, /* tracked bodies */
                                        get_alias_groups());

  const int kDim = get_robot().get_num_positions();
  for (int i = 0; i < kDim; i++) {
    plan.mutable_desired_position()[i] = state_d->GetAtIndex(i);
    plan.mutable_desired_velocity()[i] = state_d->GetAtIndex(i + kDim);
    plan.mutable_desired_acceleration()[i] = acc_d->GetAtIndex(i);
  }

  // Update desired accelerations.
  qp_input.mutable_desired_dof_motions().mutable_values() =
      plan.ComputeTargetAcceleration(robot_status->position(),
                                     robot_status->velocity());

  // Generates debugging info.
  lcmt_plan_eval_debug_info& debug =
      state->get_mutable_abstract_state()
          ->get_mutable_abstract_state(kAbsStateIdxDebug)
          .GetMutableValue<lcmt_plan_eval_debug_info>();
  debug.timestamp = static_cast<int64_t>(context.get_time() * 1e6);
  debug.num_dof = kDim;
  debug.dof_names.resize(kDim);
  debug.nominal_q.resize(kDim);
  debug.nominal_v.resize(kDim);
  debug.nominal_vd.resize(kDim);

  for (int i = 0; i < kDim; i++) {
    debug.dof_names[i] = get_robot().get_position_name(i);
    debug.nominal_q[i] = plan.desired_position()[i];
    debug.nominal_v[i] = plan.desired_velocity()[i];
    debug.nominal_vd[i] = plan.desired_acceleration()[i];
  }
}

std::unique_ptr<systems::AbstractState>
ManipulatorPlanEvalSystem::AllocateAbstractState() const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals(3);

  abstract_vals[get_abstract_state_index_plan()] =
      std::unique_ptr<systems::AbstractValue>(
          new systems::Value<VectorSetpoint<double>>(VectorSetpoint<double>()));

  abstract_vals[get_abstract_state_index_qp_input()] =
      std::unique_ptr<systems::AbstractValue>(new systems::Value<QpInput>(
          get_paramset().MakeQpInput({}, /* contacts */
                                     {}, /* tracked bodies */
                                     get_alias_groups())));

  abstract_vals[kAbsStateIdxDebug] = std::unique_ptr<systems::AbstractValue>(
      new systems::Value<lcmt_plan_eval_debug_info>(
          lcmt_plan_eval_debug_info()));

  return std::make_unique<systems::AbstractState>(std::move(abstract_vals));
}

std::unique_ptr<systems::AbstractValue>
ManipulatorPlanEvalSystem::AllocateOutputAbstract(
    const systems::OutputPortDescriptor<double>& descriptor) const {
  if (descriptor.get_index() == get_output_port_index_qp_input()) {
    return PlanEvalBaseSystem::AllocateOutputAbstract(descriptor);
  } else if (descriptor.get_index() == output_port_index_debug_info_) {
    return systems::AbstractValue::Make<lcmt_plan_eval_debug_info>(
        lcmt_plan_eval_debug_info());
  } else {
    DRAKE_DEMAND(false);
    return nullptr;
  }
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
