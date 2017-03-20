#include "drake/examples/QPInverseDynamicsForHumanoids/system/humanoid_plan_eval_system.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_path.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"

#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/humanoid_walking_plan.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/humanoid_manipulation_plan.h"

#include "robotlocomotion/robot_plan_t.hpp"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/util/lcmUtil.h"

#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/clone_only_value.h"

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

HumanoidPlanEvalSystem::HumanoidPlanEvalSystem(
    const RigidBodyTree<double>& robot,
    const std::string& alias_groups_file_name,
    const std::string& param_file_name, double dt)
    : PlanEvalBaseSystem(robot, alias_groups_file_name, param_file_name, dt),
      abs_state_index_plan_(0) {
  set_name("HumanoidPlanEval");
}

void HumanoidPlanEvalSystem::DoExtendedCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {}

std::unique_ptr<systems::AbstractValue>
HumanoidPlanEvalSystem::ExtendedAllocateOutputAbstract(
    const systems::OutputPortDescriptor<double>& descriptor) const {
  DRAKE_ABORT_MSG(
      "HumanoidPlanEvalSystem does not have additional abstract output ports.");
}

void HumanoidPlanEvalSystem::DoExtendedCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  // Gets the robot state from input.
  const HumanoidStatus* robot_status = EvalInputValue<HumanoidStatus>(
      context, get_input_port_humanoid_status().get_index());

  // HACK simulate swapping plans.
  // Should implement a set abs val func.
  if (context.get_time() > 2 && !replaned_) {
    systems::CloneOnlyValue<GenericPlan<double>> walking_plan(std::make_unique<HumanoidWalkingPlan<double>>());
    systems::CloneOnlyValue<GenericPlan<double>>& plan_ptr =
      state->get_mutable_abstract_state()->
      get_mutable_value(abs_state_index_plan_).
      GetMutableValue<systems::CloneOnlyValue<GenericPlan<double>>>();
    plan_ptr = walking_plan;
    std::cout << "Changed to walking plan\n";
    replaned_ = true;

    plan_ptr.get_value()->Initialize(*robot_status, get_paramset(), get_alias_groups());
    plan_ptr.get_value()->HandlePlanMessage(*robot_status, get_paramset(), get_alias_groups(), nullptr, 0);
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
HumanoidPlanEvalSystem::ExtendedAllocateAbstractState() const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals(
      get_num_extended_abstract_states());
  abstract_vals[abs_state_index_plan_] =
      systems::AbstractValue::Make<systems::CloneOnlyValue<GenericPlan<double>>>(
          systems::CloneOnlyValue<GenericPlan<double>>(
              //std::make_unique<HumanoidWalkingPlan<double>>()));
              std::make_unique<HumanoidManipulationPlan<double>>()));
  return abstract_vals;
}

bot_core::robot_state_t q_to_robot_state_t(const RigidBodyTree<double>& robot, const VectorX<double>& q, double time) {
  bot_core::robot_state_t msg;

  msg.utime = static_cast<int64_t>(time * 1e6);

  Isometry3<double> pose;
  pose.translation() = q.head<3>();
  pose.linear() = math::rpy2rotmat(q.segment<3>(3));
  EncodePose(pose, msg.pose);

  EncodeTwist(Vector6<double>::Zero(), msg.twist);

  msg.num_joints = robot.get_num_positions() - 6;
  msg.joint_name.resize(msg.num_joints);
  msg.joint_position.resize(msg.num_joints);
  msg.joint_velocity.resize(msg.num_joints, 0);
  msg.joint_effort.resize(msg.num_joints, 0);

  for (int i = 6; i < robot.get_num_positions(); ++i) {
    msg.joint_name[i - 6] = robot.get_position_name(i);
    msg.joint_position[i - 6] = q[i];
  }

  return msg;
}

robotlocomotion::robot_plan_t make_fake_manip_plan(const HumanoidStatus& robot_status) {
  robotlocomotion::robot_plan_t msg;

  msg.robot_name = "robot";
  msg.num_states = 3;
  msg.plan.resize(msg.num_states);
  msg.plan_info.resize(msg.num_states, 1);
  msg.num_bytes = 0;
  msg.matlab_data.resize(0);

  VectorX<double> q_d = robot_status.position();
  msg.plan[0] = q_to_robot_state_t(robot_status.robot(), q_d, 0);

  q_d[10] += 1;
  msg.plan[1] = q_to_robot_state_t(robot_status.robot(), q_d, 1);

  msg.plan[2] = q_to_robot_state_t(robot_status.robot(), q_d, 1.1);

  return msg;
}

void HumanoidPlanEvalSystem::DoInitializePlan(const HumanoidStatus& current_status,
                                              systems::State<double>* state) {
  GenericPlan<double>* plan = get_mutable_plan<GenericPlan<double>>(state, abs_state_index_plan_);
  plan->Initialize(current_status, get_paramset(), get_alias_groups());

  robotlocomotion::robot_plan_t msg = make_fake_manip_plan(current_status);
  std::vector<uint8_t> raw(msg.getEncodedSize());
  msg.encode(raw.data(), 0, raw.size());

  plan->HandlePlanMessage(current_status, get_paramset(), get_alias_groups(), raw.data(), raw.size());

  QpInput& qp_input = get_mutable_qp_input(state);
  plan->UpdateQpInput(current_status, get_paramset(), get_alias_groups(), &qp_input);
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
