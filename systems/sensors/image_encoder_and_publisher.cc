#include "apps/simulations/image_encoder_and_publisher.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"

#include "robotlocomotion/image_array_t.hpp"

namespace anzu {
namespace apps {
namespace simulation {

ImageEncoderAndPublisher::ImageEncoderAndPublisher(
    const std::string& image_channel_name, double period,
    drake::lcm::DrakeLcmInterface* lcm, bool do_compress)
    : image_channel_name_{image_channel_name},
      lcm_{lcm},
      do_compress_{do_compress} {
  this->DeclarePeriodicPublish(period);
}

void ImageEncoderAndPublisher::DoPublish(
    const drake::systems::Context<double>& context,
    const std::vector<const drake::systems::PublishEvent<double>*>&) const {
  std::vector<std::unique_ptr<drake::systems::AbstractValue>> images;
  for (int i = 0; i < get_num_input_ports(); i++) {
    const drake::systems::AbstractValue* image_value =
        this->EvalAbstractInput(context, i);
    images.emplace_back(image_value->Clone());
  }

  std::lock_guard<std::mutex> lock(image_lock_);
  images_ = std::move(images);
  image_time_ = context.get_time();
}

ImageEncoderAndPublisher::~ImageEncoderAndPublisher() {
  Stop();
}

void ImageEncoderAndPublisher::Start() {
  if (run_) return;
  run_ = true;
  publish_thread_ = std::thread(&ImageEncoderAndPublisher::PublishThreadLoop, this);
}

void ImageEncoderAndPublisher::Stop() {
  if (!run_) return;
  run_ = false;
  publish_thread_.join();
}

void ImageEncoderAndPublisher::PublishThreadLoop() {
  std::vector<std::unique_ptr<drake::systems::AbstractValue>> images;
  double time{};

  robotlocomotion::image_array_t msg{};
  while (run_) {
    // Get the images first.
    {
      std::lock_guard<std::mutex> lock(image_lock_);
      if (!images_.empty()) {
        images = std::move(images_);
        time = image_time_;
      } else {
        continue;
      }
    }

    DRAKE_THROW_UNLESS(images.size() == input_port_pixel_type_.size());

    const int num_images = static_cast<int>(input_port_pixel_type_.size());
    msg.header.utime = static_cast<int64_t>(time * 1e6);
    msg.header.frame_name.resize(num_images);
    msg.num_images = num_images;
    msg.images.resize(num_images);

    for (int i = 0; i < num_images; i++) {
      drake::systems::sensors::ImageToLcmImageArrayT::PackImageToLcmImageT(
          *images[i], input_port_pixel_type_[i], msg.header.utime,
          get_input_port(i).get_name(), &(msg.images[i]), do_compress_);
    }

    std::vector<uint8_t> bytes(msg.getEncodedSize());
    msg.encode(bytes.data(), 0, bytes.size());
    lcm_->Publish(image_channel_name_, bytes.data(), bytes.size(),
                  drake::nullopt);
  }
}

}  // namespace simulation
}  // namespace apps
}  // namespace anzu
