#pragma once

#include <string>
#include <vector>

#include <lcm/lcm-cpp.hpp>

#include "drake/common/text_logging.h"

namespace drake {
namespace examples {
namespace manipulation_station {

/// Provides simple synchronization to external scripts, e.g.
/// `capture_image.py`.
template <typename Message>
class LcmBlockingTrigger {
 public:
  LcmBlockingTrigger(const std::string& channel, const std::string& lcm_url)
      : channel_(channel), channel_ack_(channel + "_ACK"), lcm_(lcm_url) {
    // Since this object owns the LCM object, we do not need to bother
    // unsubscribing at destruction.
    auto sub =
        lcm_.subscribe(channel_ack_, &LcmBlockingTrigger::AckCallback, this);
    sub->setQueueCapacity(1);
  }

  void PublishTriggerAndWaitForAck(const Message& input = {},
                                   Message* output = nullptr) {
    // Data doesn't actually matter.
    std::vector<uint8_t> bytes(input.getEncodedSize());
    input.encode(bytes.data(), 0, bytes.size());
    drake::log()->info("[ trigger ] Published {}", channel_);
    lcm_.publish(channel_, bytes.data(), bytes.size());
    WaitForAck(output);
  }

  void WaitForAck(Message* output = nullptr) {
    ack_ = false;
    drake::log()->info("[ trigger ] Wait for ack...");
    while (!ack_) {
      lcm_.handle();
    }
    drake::log()->info("[ trigger ] - Received");
    if (output) {
      *output = msg_;
    }
  }

 private:
  void AckCallback(const ::lcm::ReceiveBuffer*, const std::string&,
                   const Message* msg) {
    ack_ = true;
    msg_ = *msg;
  }

  std::string channel_;
  std::string channel_ack_;
  ::lcm::LCM lcm_;
  bool ack_{false};
  Message msg_;
};

}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake
