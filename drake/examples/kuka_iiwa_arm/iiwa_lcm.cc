#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"

#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

using systems::BasicVector;
using systems::Context;
using systems::DiscreteState;
using systems::SystemOutput;

static const int kNumJoints = 7;
template <typename Scalar, int dim>
using Vector = Eigen::Matrix<Scalar, dim, 1>;

IiwaDebugMsgGen::IiwaDebugMsgGen() {
  in_idx_cmd_ = DeclareInputPort(systems::kVectorValued, kNumJoints * 2).get_index();
  in_idx_state_ = DeclareInputPort(systems::kVectorValued, kNumJoints * 2).get_index();

  out_idx_msg_ = DeclareAbstractOutputPort().get_index();
}

void IiwaDebugMsgGen::DoCalcOutput(const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  lcmt_iiwa_status& msg = output->GetMutableData(out_idx_msg_)
          ->GetMutableValue<lcmt_iiwa_status>();

  const systems::BasicVector<double>* state =
      this->EvalVectorInput(context, in_idx_state_);

  const systems::BasicVector<double>* command =
      this->EvalVectorInput(context, in_idx_cmd_);

  msg.utime = context.get_time() * 1e6;
  msg.num_joints = kNumJoints;
  msg.joint_position_measured.resize(msg.num_joints, 0);
  msg.joint_position_commanded.resize(msg.num_joints, 0);
  msg.joint_position_ipo.resize(msg.num_joints, 0);
  msg.joint_torque_measured.resize(msg.num_joints, 0);
  msg.joint_torque_commanded.resize(msg.num_joints, 0);
  msg.joint_torque_external.resize(msg.num_joints, 0);

  for (int i = 0; i < kNumJoints; i++) {
    msg.joint_position_measured[i] = state->GetAtIndex(i);
    msg.joint_position_commanded[i] = command->GetAtIndex(i);

    msg.joint_torque_measured[i] = state->GetAtIndex(i + 7);
    msg.joint_torque_commanded[i] = command->GetAtIndex(i + 7);
  }
}

std::unique_ptr<systems::AbstractValue> IiwaDebugMsgGen::AllocateOutputAbstract(
    const systems::OutputPortDescriptor<double>& descriptor) const {

  lcmt_iiwa_status msg{};
  msg.num_joints = kNumJoints;
  msg.joint_position_measured.resize(msg.num_joints, 0);
  msg.joint_position_commanded.resize(msg.num_joints, 0);
  msg.joint_position_ipo.resize(msg.num_joints, 0);
  msg.joint_torque_measured.resize(msg.num_joints, 0);
  msg.joint_torque_commanded.resize(msg.num_joints, 0);
  msg.joint_torque_external.resize(msg.num_joints, 0);

  return systems::AbstractValue::Make<lcmt_iiwa_status>(msg);
}


// This value is chosen to match the value in getSendPeriodMilliSec()
// when initializing the FRI configuration on the iiwa's control
// cabinet.
static const double kReceiverUpdatePeriod = 0.005;

IiwaCommandReceiver::IiwaCommandReceiver() {
  this->DeclareAbstractInputPort();
  this->DeclareOutputPort(systems::kVectorValued, kNumJoints * 2);
  this->DeclareDiscreteUpdatePeriodSec(kReceiverUpdatePeriod);
  // command
  // last message time
  this->DeclareDiscreteState(kNumJoints * 2 + 1);

  set_name("CMD_RECEIVER");
}

void IiwaCommandReceiver::set_initial_position(
    Context<double>* context,
    const Eigen::Ref<const VectorX<double>> x) const {
  auto state_value =
      context->get_mutable_discrete_state(0)->get_mutable_value();
  DRAKE_DEMAND(x.size() == kNumJoints);
  // Init the last message receive time to -1
  state_value << x, Vector<double, kNumJoints>::Zero(), -1;
}

void IiwaCommandReceiver::DoCalcDiscreteVariableUpdates(
    const Context<double>& context,
    DiscreteState<double>* discrete_state) const {
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& command = input->GetValue<lcmt_iiwa_command>();
  // TODO(sam.creasey) Support torque control.
  DRAKE_ASSERT(command.num_torques == 0);

  // If we're using a default constructed message (haven't received
  // a command yet), keep using the initial state.
  if (command.num_joints != 0) {
    // Gets last desired state.
    BasicVector<double>* state = discrete_state->get_mutable_discrete_state(0);
    auto state_value = state->get_mutable_value();

    // Gets the last time when a different lcm message is received.
    double last_msg_time = state_value[2 * kNumJoints];

    DRAKE_DEMAND(command.num_joints == kNumJoints);
    Vector<double, kNumJoints> pos_d;

    for (int i = 0; i < command.num_joints; ++i) {
      pos_d(i) = command.joint_position[i];
    }

    double cur_time = command.utime / 1e6;
    // first real message
    if (last_msg_time == -1) {
      state_value << pos_d, Vector<double, kNumJoints>::Zero(), cur_time;
    }

    if (cur_time != last_msg_time) {
      Vector<double, kNumJoints> vel_d =
          (pos_d - state_value.head<kNumJoints>()) / (cur_time - last_msg_time);
      state_value << pos_d, vel_d, cur_time;
    }
  }
}

void IiwaCommandReceiver::DoCalcOutput(const Context<double>& context,
                                       SystemOutput<double>* output) const {
  Eigen::VectorBlock<VectorX<double>> output_vec =
      this->GetMutableOutputVector(output, 0);
  output_vec = context.get_discrete_state(0)->get_value();
}

IiwaStatusSender::IiwaStatusSender() {
  this->DeclareInputPort(systems::kVectorValued, kNumJoints * 2);
  this->DeclareInputPort(systems::kVectorValued, kNumJoints * 2);
  this->DeclareAbstractOutputPort();
}

std::unique_ptr<systems::SystemOutput<double>>
IiwaStatusSender::AllocateOutput(
    const systems::Context<double>& context) const {
  auto output = std::make_unique<systems::LeafSystemOutput<double>>();
  lcmt_iiwa_status msg{};
  msg.num_joints = kNumJoints;
  msg.joint_position_measured.resize(msg.num_joints, 0);
  msg.joint_position_commanded.resize(msg.num_joints, 0);
  msg.joint_position_ipo.resize(msg.num_joints, 0);
  msg.joint_torque_measured.resize(msg.num_joints, 0);
  msg.joint_torque_commanded.resize(msg.num_joints, 0);
  msg.joint_torque_external.resize(msg.num_joints, 0);

  output->get_mutable_ports()->emplace_back(
      std::make_unique<systems::OutputPort>(
          std::make_unique<systems::Value<lcmt_iiwa_status>>(msg)));
  return std::unique_ptr<SystemOutput<double>>(output.release());
}

void IiwaStatusSender::DoCalcOutput(
    const Context<double>& context, SystemOutput<double>* output) const {
  systems::AbstractValue* mutable_data = output->GetMutableData(0);
  lcmt_iiwa_status& status =
      mutable_data->GetMutableValue<lcmt_iiwa_status>();

  status.utime = context.get_time() * 1e6;
  const systems::BasicVector<double>* command =
      this->EvalVectorInput(context, 0);
  const systems::BasicVector<double>* state =
      this->EvalVectorInput(context, 1);
  for (int i = 0; i < kNumJoints; ++i) {
    status.joint_position_measured[i] = state->GetAtIndex(i);
    status.joint_position_commanded[i] = command->GetAtIndex(i);
  }
}


}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
