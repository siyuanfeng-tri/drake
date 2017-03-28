#pragma once

#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * This is a very crude first pass to implement a loop that's driven by a lcm
 * message. Moreover, context time is explicitly slaved to whatever time is in
 * the received lcm message, and most likely some states are also slaved to the
 * incoming message implicitly. This is intended to implement a control loop
 * that handles a state message from the network, computes some numbers and then
 * sends some command out.
 *
 * This class uses the Simulator class internally for event handling
 * (kPublishAction, kDiscreteUpdateAction, kUnrestrictedUpdateAction) and
 * state "integration" (e.g. I term in a PID). Since this class is intended to
 * solve a more specific subset of problems, the user needs to be mindful of how
 * their system is configured, especially about event timing. The main message
 * handling loop is roughly:
 * <pre>
 * msg = wait_for_message("channel");
 * stepper.context.Initliaze(msg.time, msg);
 * while(true) {
 *   msg = wait_for_message("channel");
 *   stepper.StepTo(msg.time);
 *   if (publish) {
 *     system.Publish(stepper.context);
 *   }
 * }
 *
 *
 * There are a couple assumptions in this implementation.
 * 1. The loop is blocked only on one lcm message.
 * 2. It's pointless to recompute without a new lcm message (thus the handler
 * thread is de-scheduled).
 * 3. The message needs to have a utime field that is set properly, in units of
 * microseconds..
 * 4. The computation for the given system should be faster than the incoming
 * message rate.
 *
 * Subtle things about lcm subscribers:
 * LcmSubscriberSystem's output is not guaranteed to be the same given the same
 * context. E.g. if a lcm messages comes in between two calls to CalcOutput, the
 * second call would return the new message while the "state" in the context
 * remains the same. It is thus recommended that all the lcm message sources are
 * connected with a ZOH block that only pulls from the actual subscriber when
 * time changes.
 */
template <typename T>
class LcmDrivenLoop {
 public:
  // The max count for semaphore_ needs to be 1. Otherwise the behavior is not
  // correct if we handle messages slower than the incoming rate.
  LcmDrivenLoop() : semaphore_(1) {}

  void Initialize(const System<T>& system, std::unique_ptr<Context<T>> context,
                  LcmSubscriberSystem* driving_subscriber) {
    DRAKE_DEMAND(driving_subscriber != nullptr);

    system_ = &system;
    driving_sub_ = driving_subscriber;
    stepper_ = std::make_unique<Simulator<T>>(*system_, std::move(context));

    driving_sub_->set_notification(&semaphore_);

    sub_context_ = driving_sub_->CreateDefaultContext();
    sub_output_ = driving_sub_->AllocateOutput(*sub_context_);

    stepper_->set_publish_every_time_step(false);

    lcm_.StartReceiveThread();
  }

  template <typename MessageType>
  void Run() {
    // Wake up for the first message in driving_sub_, so that we can initialize
    // the time properly.
    semaphore_.wait();

    // Explicitly eval the driving subscriber to get the lcm message.
    driving_sub_->CalcOutput(*sub_context_, sub_output_.get());
    // Ok, this is really screwy. This won't compile for any message that
    // doesn't have a "utime" field.
    const MessageType& msg = sub_output_->get_data(0)->GetValue<MessageType>();
    T msg_time = static_cast<T>(msg.utime) / 1e6;

    // Init our time to the msg time.
    stepper_->get_mutable_context()->set_time(msg_time);
    stepper_->Initialize();

    // Wall clock of the this computer when the lcm message arrived, in micro
    // seconds.
    T wall_clock_now = sub_output_->get_vector_data(1)->GetAtIndex(0);
    T wall_clock_prev = wall_clock_now;
    T dt;

    while (true) {
      // Wake up only when there is a new message in driving_sub_.
      semaphore_.wait();

      driving_sub_->CalcOutput(*sub_context_, sub_output_.get());
      wall_clock_now = sub_output_->get_vector_data(1)->GetAtIndex(0);
      dt = wall_clock_now - wall_clock_prev;
      wall_clock_prev = wall_clock_now;
      std::cout << "dt(wall clock): " << dt / 1e6 << std::endl;

      // Get the lcm message.
      const MessageType& msg =
          sub_output_->get_data(0)->GetValue<MessageType>();
      msg_time = static_cast<T>(msg.utime) / 1e6;
      std::cout << "msg t: " << msg_time << std::endl;
      std::cout << "msg dt: " << msg_time - stepper_->get_context().get_time()
                << std::endl;

      // Do everything else normally.
      stepper_->StepTo(msg_time);

      // Explicitly publish after we are done with all the intermediate
      // computation.
      if (publish_on_every_received_message_) {
        system_->Publish(stepper_->get_context());
      }
    }
  }

  /**
   * If @p flag is set to true, the main loop will explicitly have all
   * subsystems publish after it handles one driving message. The user should
   * either make sure ((no subsystems have DeclarePublishPeriodSec() or
   * set_publish_period() calls) and @p flag being ture), or
   * ((every subsytem have there desired publish rates set properly with
   * DeclarePublishPeriodSec() or set_publish_period() calls) and @p flag
   * being false) to avoid repeated publishing. The former corresponds to
   * "publish everything whenever a new message has been handled". The
   * latrer roughly corresponds to "All subsystems publish on their declared
   * rates." However, this strongly depends on the timing of the driving
   * message and computation in message handling, and the publish is likely
   * to be not uniform. Suppose the driving message comes at 1 hz, and the
   * message handlers take very little time to handle one message, and are
   * declared to be publishing at 200hz. What you should oberserve is that no
   * messages will be published for 1 second, and 200 messages will be
   * published almost at once. This is most likely not the intended behavior,
   * and indicates some basic assumptions are probably wrong.
   *
   * The first setup is recommended, because it fits the intuitive "one state
   * in, one control out" picture better. The second setup is also fine given
   * everything happens "fast enough".
   */
  void set_publish_on_every_received_message(bool flag) {
    publish_on_every_received_message_ = flag;
  }

  drake::lcm::DrakeLcm* get_mutable_lcm() { return &lcm_; }

  Context<T>* get_mutable_context() {
    return stepper_->get_mutable_context();
  }

 private:
  // The lcm interface for publishing and subscribing.
  drake::lcm::DrakeLcm lcm_;

  bool publish_on_every_received_message_{true};

  // The semaphore that blocks the thread that calls Run() until a message
  // triggers driving_sub_'s message handler which wakes up this semaphore.
  Semaphore semaphore_;

  // The system that does stuff.
  const System<T>* system_;

  // THE message handler. stepper_.context's time will be latched to whatever
  // time is being subscribed in driving_sub_.
  LcmSubscriberSystem* driving_sub_;

  // Reusing the simulator to manage event handling and state progression.
  std::unique_ptr<Simulator<T>> stepper_;

  // Separate context and output port just to get the time out of
  // driving_sub_.. I can't think of a general way to get the correct sub
  // context out of stepper_.context if driving_sub_ is buried deep down
  // somewhere in system_.
  std::unique_ptr<Context<double>> sub_context_;
  std::unique_ptr<SystemOutput<double>> sub_output_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
