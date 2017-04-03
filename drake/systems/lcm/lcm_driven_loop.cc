#include "drake/systems/lcm/lcm_driven_loop.h"

namespace drake {
namespace systems {
namespace lcm {

LcmDrivenLoop::LcmDrivenLoop(
    const System<double>& system, const LcmSubscriberSystem& driving_subscriber,
    std::unique_ptr<Context<double>> context, drake::lcm::DrakeLcm* lcm,
    std::unique_ptr<LcmMessageToTimeInterface> time_converter)
    : system_(system),
      driving_sub_(driving_subscriber),
      lcm_(lcm),
      time_converter_(std::move(time_converter)),
      stepper_(
          std::make_unique<Simulator<double>>(system_, std::move(context))) {
  DRAKE_DEMAND(lcm != nullptr);
  DRAKE_DEMAND(time_converter_ != nullptr);

  // Allocates extra context and output just for the driving subscriber, so
  // that this can explicitly query the message.
  sub_context_ = driving_sub_.CreateDefaultContext();
  sub_output_ = driving_sub_.AllocateOutput(*sub_context_);
  sub_swap_state_ = sub_context_->CloneState();

  // Disables simulator's publish on its internal time step.
  stepper_->set_publish_every_time_step(false);

  // Starts the subscribing thread.
  lcm_->StartReceiveThread();
}

const AbstractValue& LcmDrivenLoop::WaitForMessage() {
  UpdateActions<double> actions;

  message_count_ = driving_sub_.WaitForMessage(message_count_);

  driving_sub_.CalcNextUpdateTime(*sub_context_, &actions);
  driving_sub_.CalcUnrestrictedUpdate(*sub_context_, actions.events.front(), sub_swap_state_.get());
  sub_context_->get_mutable_state()->CopyFrom(*sub_swap_state_);

  driving_sub_.CalcOutput(*sub_context_, sub_output_.get());
  return *(sub_output_->get_data(0));
}

void LcmDrivenLoop::RunAssumingInitializedTo(double stop_time) {
  double msg_time;
  stepper_->Initialize();

  while (true) {
    msg_time = time_converter_->GetTimeInSeconds(WaitForMessage());
    std::cout << "t now: " << stepper_->get_context().get_time() << "\n";
    std::cout << "t nxt: " << msg_time << "\n";

    if (msg_time >= stop_time) break;

    stepper_->StepTo(msg_time);

    // Explicitly publish after we are done with all the intermediate
    // computation.
    if (publish_on_every_received_message_) {
      system_.Publish(stepper_->get_context());
    }
  }
}

void LcmDrivenLoop::RunWithDefaultInitializationTo(double stop_time) {
  const AbstractValue& first_msg = WaitForMessage();
  double msg_time = time_converter_->GetTimeInSeconds(first_msg);
  std::cout << "t0 " << msg_time << std::endl;
  // Inits context time to the msg time.
  stepper_->get_mutable_context()->set_time(msg_time);

  RunAssumingInitializedTo(stop_time);
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
