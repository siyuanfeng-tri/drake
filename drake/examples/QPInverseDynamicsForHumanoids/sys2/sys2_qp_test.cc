#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/translator_between_lcmt_drake_signal.h"
#include "drake/lcmt_drake_signal.hpp"

#include "drake/systems/lcm/lcm_receive_thread.h"


#include "sys2_qp.h"

class MessagePublisher {
 public:
  MessagePublisher(const std::string& channel_name, ::lcm::LCM* lcm, int size)
      : channel_name_(channel_name), lcm_(lcm) {
    if (size == drake::systems::System2QP::num_states)
      type_ = 0;
    else if (size == drake::systems::System2QP::num_qp_input)
      type_ = 1;
    else
      type_ = 2;

    message_.dim = size;
    message_.val.resize(message_.dim);
    message_.coord.resize(message_.dim);
    for (int ii = 0; ii < message_.dim; ++ii) {
      message_.val[ii] = ii;
      message_.coord[ii] = "coord_" + std::to_string(ii);
    }
    message_.timestamp = 0;
  }

  void Start() {
    thread_.reset(new std::thread(&MessagePublisher::DoPublish, this));
  }

  void Stop() {
    stop_ = true;
    thread_->join();
  }

 private:
  void DoPublish() {
    int ctr = 0;
    while (!stop_) {
      for (int i = 0; i < message_.dim; i++) {
        if (type_ == 1)
          message_.val[i] = (double)ctr / 10;
        else if (type_ == 0)
          message_.val[i] = (double)ctr / 100;
        else
          message_.val[i] = ctr;
      }
      ctr++;

      lcm_->publish(channel_name_, &message_);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  }

  const std::string channel_name_;

  ::lcm::LCM* lcm_;

  int type_;

  drake::lcmt_drake_signal message_;

  std::atomic<bool> stop_{false};

  std::unique_ptr<std::thread> thread_;
};

void test_lcm_sub() {
  lcm::LCM lcm;
  // lcm receiver
  drake::systems::lcm::LcmReceiveThread lcm_receive_thread(&lcm);

  // lcm pub
  MessagePublisher pub("test", &lcm, 3);
  pub.Start();

  std::string channel = "test";

  const drake::systems::lcm::TranslatorBetweenLcmtDrakeSignal translator(3);

  drake::systems::lcm::LcmSubscriberSystem dut(channel, translator, &lcm);
  std::unique_ptr<drake::systems::ContextBase<double>> context = dut.CreateDefaultContext();
  std::unique_ptr<drake::systems::SystemOutput<double>> output = dut.AllocateOutput(*context);

  Eigen::VectorXd mlgb;

  while (true) {
    dut.EvalOutput(*context.get(), output.get());
    // Gets the output of the LcmSubscriberSystem.
    // Downcasts the output vector to be a pointer to a BasicVector.
    const drake::systems::BasicVector<double>& basic_vector =
        dynamic_cast<const drake::systems::BasicVector<double>&>(*output->get_port(0).get_vector_data());

    mlgb = basic_vector.get_value();
    std::cout << mlgb.transpose() << std::endl;
  }
}


int main() {
  lcm::LCM lcm;
  // lcm receiver
  drake::systems::lcm::LcmReceiveThread lcm_receive_thread(&lcm);

  // lcm pub
  MessagePublisher state_pub("state", &lcm, drake::systems::System2QP::num_states);
  state_pub.Start();
  MessagePublisher qp_input_pub("qp_input", &lcm, drake::systems::System2QP::num_qp_input);
  qp_input_pub.Start();
  std::cout << "pubs started\n";

  drake::systems::testQP(lcm);
  return 0;
}
