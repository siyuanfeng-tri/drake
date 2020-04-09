#include "drake/examples/manipulation_station/point_cloud_visualizer.h"

namespace drake {
namespace examples {
namespace manipulation_station {

PointCloudVisualizer::PointCloudVisualizer(
    const geometry::render::DepthCameraProperties& cam_prop,
    const multibody::MultibodyPlant<double>* plant,
    const multibody::Frame<double>* camera_frame)
    : cam_prop_(cam_prop), plant_(*plant), camera_frame_(*camera_frame) {
  auto context = plant_.CreateDefaultContext();
  Value<systems::Context<double>> val{std::move(context)};
  this->DeclareAbstractInputPort("kinematics", val);

  this->DeclareAbstractInputPort("depth_image",
                                 Value<systems::sensors::ImageDepth16U>());
  this->DeclareAbstractInputPort("color_image",
                                 Value<systems::sensors::ImageRgba8U>());

  this->DeclareAbstractOutputPort(
      "point_cloud_lcm_msg", bot_core::pointcloud_t{},
      &PointCloudVisualizer::CalcPointCloud, {this->all_input_ports_ticket()});

  fx_ = cam_prop.height / 2 / std::tan(cam_prop.fov_y / 2);
  fy_ = cam_prop.height / 2 / std::tan(cam_prop.fov_y / 2);
  ppx_ = cam_prop_.width / 2;
  ppy_ = cam_prop_.height / 2;
}

Eigen::Vector3d PointCloudVisualizer::BackProject(const Eigen::Vector2d& uv,
                                                  double depth) const {
  double x = (uv[0] - ppx_) / fx_;
  double y = (uv[1] - ppy_) / fy_;
  return Eigen::Vector3d(depth * x, depth * y, depth);
}

void PointCloudVisualizer::CalcPointCloud(
    const systems::Context<double>& context,
    bot_core::pointcloud_t* message) const {
  const auto& kinematics =
      get_kinematics_input().Eval<systems::Context<double>>(context);
  const auto& rgb_img =
      get_rgb_image_input().Eval<systems::sensors::ImageRgba8U>(context);
  const auto& depth_img =
      get_depth_image_input().Eval<systems::sensors::ImageDepth16U>(context);

  const int height = rgb_img.height();
  const int width = rgb_img.width();
  const int num_pixels = height * width;

  auto X_WC = plant_.CalcRelativeTransform(kinematics, plant_.world_frame(),
                                           camera_frame_);

  message->points.clear();
  message->frame_id = "world";
  message->n_points = num_pixels;
  message->points.resize(message->n_points);
  // See: director.drakevisualizer, DrakeVisualier.onPointCloud
  message->n_channels = 3;
  message->channel_names = {"r", "g", "b"};
  message->channels.resize(3, std::vector<float>(message->n_points));

  int pixel = 0;
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      double depth = static_cast<double>(*depth_img.at(x, y)) / 1000.;
      const Eigen::Vector3f p_W =
          (X_WC * BackProject(Eigen::Vector2d(x, y), depth)).cast<float>();
      // Copy color
      for (int c = 0; c < 3; c++) {
        message->channels[c][pixel] = rgb_img.at(x, y)[c] / 255.;
      }
      // Copy xyz.
      message->points[pixel] = {p_W[0], p_W[1], p_W[2]};
      pixel++;
    }
  }
}

}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake
