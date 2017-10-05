#include "drake/common/find_resource.h"
#include "drake/examples/humanoid_controller/humanoid_controller.h"
#include "drake/examples/valkyrie/valkyrie_constants.h"
#include "drake/manipulation/util/robot_state_msg_translator.h"
#include "drake/systems/lcm/lcm_driven_loop.h"

namespace drake {
namespace examples {
namespace humanoid_controller {

std::unique_ptr<systems::AbstractValue> WaitForMessage(
    const systems::lcm::LcmSubscriberSystem& sub,
    const systems::Context<double>& sub_context) {
  auto sub_events = sub.AllocateCompositeEventCollection();
  auto tmp_state = sub_context.CloneState();
  auto sub_output = sub.AllocateOutput(sub_context);
  sub.WaitForMessage(sub.GetMessageCount(sub_context));
  sub.CalcNextUpdateTime(sub_context, sub_events.get());

  // If driving_sub_.WaitForMessage() returned, a message should be received
  // and an event should be queued by driving_sub_.CalcNextUpdateTime().
  if (sub_events->HasUnrestrictedUpdateEvents()) {
    sub.CalcUnrestrictedUpdate(sub_context,
        sub_events->get_unrestricted_update_events(),
        tmp_state.get());
  } else {
    DRAKE_DEMAND(false);
  }

  auto new_context = sub_context.Clone();
  new_context->get_mutable_state()->CopyFrom(*tmp_state);

  sub.CalcOutput(*new_context, sub_output.get());
  return sub_output->get_data(0)->Clone();
}

// This is an example qp based inverse dynamics controller loop for Valkyrie
// built from the Systems blocks.
//
// The overall input and output is a LCM message of type
// bot_core::robot_state_t and bot_core::atlas_command_t.
void controller_loop() {
  const std::string kModelFileName = FindResourceOrThrow(
      "drake/examples/valkyrie/urdf/urdf/"
      "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf");
  const std::string kAliasGroupPath = FindResourceOrThrow(
      "drake/examples/humanoid_controller/"
      "config/valkyrie.alias_groups");
  const std::string kControlConfigPath = FindResourceOrThrow(
      "drake/examples/humanoid_controller/"
      "config/valkyrie.id_controller_config");

  drake::lcm::DrakeLcm lcm;
  HumanoidController valkyrie_controller(kModelFileName, kControlConfigPath,
                                         kAliasGroupPath, &lcm);
  HumanoidPlanEvalSystem* plan_eval =
      valkyrie_controller.get_mutable_plan_eval();
  const systems::lcm::LcmSubscriberSystem& state_msg_subscriber =
      valkyrie_controller.get_state_msg_subscriber();

  systems::Simulator<double> stepper(valkyrie_controller);

  const auto& sub_context =
      valkyrie_controller.GetSubsystemContext(
          state_msg_subscriber, stepper.get_context());
  // Blocks till the first real message.
  lcm.StartReceiveThread();
  auto first_msg = WaitForMessage(
      state_msg_subscriber, sub_context);

  // Decodes the message into q and v.
  const bot_core::robot_state_t& raw_msg =
      first_msg->GetValueOrThrow<bot_core::robot_state_t>();
  RigidBodyTree<double> robot;
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      kModelFileName, multibody::joints::kRollPitchYaw, &robot);
  RigidBodyTreeAliasGroups<double> alias_groups(&robot);
  alias_groups.LoadFromFile(kAliasGroupPath);

  VectorX<double> q(robot.get_num_positions());
  VectorX<double> v(robot.get_num_velocities());
  manipulation::RobotStateLcmMessageTranslator translator(robot);
  translator.DecodeMessageKinematics(raw_msg, q, v);

  HumanoidStatus robot_status(&robot, alias_groups);
  v.setZero();
  robot_status.UpdateKinematics(0, q, v);

  // Sets plan eval's desired to the measured state.
  systems::Context<double>& plan_eval_context =
      valkyrie_controller.GetMutableSubsystemContext(
          *plan_eval, stepper.get_mutable_context());
  plan_eval->Initialize(robot_status, plan_eval_context.get_mutable_state());

  // My clock is tied to wall clock now, not to the message's clock
  // any more. this needs to be fixed some other time.
  stepper.set_target_realtime_rate(1);
  stepper.set_publish_every_time_step(false);

  // Any small epsilon should work, this magic number has to be bigger than the
  // hard coded 1e-4 number in subscriber's calc next update time. (this is pretty subtle..)
  // You can't make this too big either, because otherwise the realtime
  // factor thing will sleep until that dt. Duh!
  while (true) {
    stepper.StepTo(stepper.get_context().get_time() + 1e-3);
  }
}

}  // namespace humanoid_controller
}  // namespace examples
}  // end namespace drake

int main() { drake::examples::humanoid_controller::controller_loop(); }
