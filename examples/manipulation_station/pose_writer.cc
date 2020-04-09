#include "drake/examples/manipulation_station/pose_writer.h"

namespace drake {
namespace multibody {

PoseWriter::PoseWriter(const MultibodyPlant<double>* plant,
                       const std::vector<const Frame<double>*>& frames,
                       const std::string& file_name, double period,
                       double t_offset)
    : plant_(*plant),
      context_(plant_.CreateDefaultContext()),
      frames_(frames),
      out_(std::ofstream(file_name)) {
  this->DeclareVectorInputPort(
      "state_input",
      systems::BasicVector<double>(plant_.num_multibody_states()));
  this->set_name("pose_writer");
  this->DeclarePeriodicPublishEvent(period, t_offset, &PoseWriter::WritePose);
}

void PoseWriter::WritePose(const systems::Context<double>& context) const {
  const systems::BasicVector<double>* x = this->EvalVectorInput(context, 0);
  plant_.SetPositionsAndVelocities(context_.get(), x->get_value());

  auto write = [](const math::RigidTransform<double>& pose,
                  const std::string& name, const int count, const double time,
                  std::ofstream* out) {
    const auto quat = pose.rotation().ToQuaternion();
    const auto& trans = pose.translation();
    const std::string rgb_img_name =
        fmt::format("{count:06}_rgb.png", fmt::arg("count", count));
    const std::string depth_img_name =
        fmt::format("{count:06}_depth.png", fmt::arg("count", count));
    (*out) << count << ":\n";
    (*out) << "  " << name << ":\n";
    (*out) << "    "
           << "quaternion:\n";
    (*out) << "      "
           << "w: " << fmt::format("{:.12f}", quat.w()) << "\n";
    (*out) << "      "
           << "x: " << fmt::format("{:.12f}", quat.x()) << "\n";
    (*out) << "      "
           << "y: " << fmt::format("{:.12f}", quat.y()) << "\n";
    (*out) << "      "
           << "z: " << fmt::format("{:.12f}", quat.z()) << "\n";
    (*out) << "    "
           << "translation:\n";
    (*out) << "      "
           << "x: " << fmt::format("{:.12f}", trans.x()) << "\n";
    (*out) << "      "
           << "y: " << fmt::format("{:.12f}", trans.y()) << "\n";
    (*out) << "      "
           << "z: " << fmt::format("{:.12f}", trans.z()) << "\n";
    (*out) << "  "
           << "depth_image_filename: " << depth_img_name << "\n";
    (*out) << "  "
           << "rgb_image_filename: " << rgb_img_name << "\n";
    (*out) << "  "
           << "timestamp: " << static_cast<uint64_t>(time * 1e6) << "\n";
  };

  for (const Frame<double>* frame : frames_) {
    auto X_WF =
        plant_.CalcRelativeTransform(*context_, plant_.world_frame(), *frame);
    write(X_WF, "camera_to_world", ctr_, context.get_time(), &out_);
  }
  out_.flush();
  ctr_++;
}

}  // namespace multibody
}  // namespace drake
