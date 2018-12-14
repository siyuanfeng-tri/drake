#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/pixel_types.h"

namespace anzu {
namespace apps {
namespace simulation {

class ImageEncoderAndPublisher : public drake::systems::LeafSystem<double> {
 public:
  ImageEncoderAndPublisher(const std::string& image_channel_name, double period,
                           drake::lcm::DrakeLcmInterface* lcm,
                           bool do_compress = false);

  ~ImageEncoderAndPublisher();

  template <drake::systems::sensors::PixelType kPixelType>
  const drake::systems::InputPort<double>& DeclareImageInputPort(
      const std::string& name) {
    input_port_pixel_type_.push_back(kPixelType);
    return this->DeclareAbstractInputPort(
        name,
        drake::systems::Value<drake::systems::sensors::Image<kPixelType>>());
  }

  void Start();
  void Stop();

 private:
  void DoPublish(const drake::systems::Context<double>& context,
                 const std::vector<const drake::systems::PublishEvent<double>*>&
                     events) const;

  void PublishThreadLoop();

  const std::string image_channel_name_;
  mutable drake::lcm::DrakeLcmInterface* lcm_{};
  const bool do_compress_;

  std::vector<drake::systems::sensors::PixelType> input_port_pixel_type_{};

  std::atomic<bool> run_{false};
  std::thread publish_thread_;

  mutable std::mutex image_lock_;
  mutable std::vector<std::unique_ptr<drake::systems::AbstractValue>> images_;
  mutable double image_time_;
};

}  // namespace simulation
}  // namespace apps
}  // namespace anzu
