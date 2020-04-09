#pragma once

#include <fstream>
#include <string>
#include <vector>

#include "bot_core/pointcloud_t.hpp"

#include "drake/common/drake_copyable.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace manipulation_station {

class PointCloudVisualizer : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PointCloudVisualizer)

  /**
   * This assumes that camera's RGB and depth are co-locationg.
   */
  PointCloudVisualizer(const geometry::render::DepthCameraProperties& cam_prop,
                       const multibody::MultibodyPlant<double>* plant,
                       const multibody::Frame<double>* camera_frame);

  const systems::InputPort<double>& get_depth_image_input() const {
    return get_input_port(1);
  }

  const systems::InputPort<double>& get_rgb_image_input() const {
    return get_input_port(2);
  }

  const systems::InputPort<double>& get_kinematics_input() const {
    return get_input_port(0);
  }

 private:
  void CalcPointCloud(const systems::Context<double>& context,
                      bot_core::pointcloud_t* message) const;

  Eigen::Vector3d BackProject(const Eigen::Vector2d& uv, double z) const;

  geometry::render::DepthCameraProperties cam_prop_;
  const multibody::MultibodyPlant<double>& plant_;
  const multibody::Frame<double>& camera_frame_;
  double fy_{};
  double fx_{};
  double ppy_{};
  double ppx_{};
};

}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake
