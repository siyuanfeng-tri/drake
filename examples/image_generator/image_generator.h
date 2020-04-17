#pragma once

#include "drake/common/eigen_types.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"

#include "drake/examples/manipulation_station/pose_writer.h"
#include "drake/systems/sensors/image_writer.h"
#include "drake/systems/sensors/mask_converter.h"

namespace drake {
namespace examples {
namespace image_generator {

class ImageGenerator : public systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImageGenerator)
  // if mask_value_source == -1, mask value is coming from the texture.
  // otherwise, mask for obj will be set to mask_value_source.
  ImageGenerator(const std::string& model_path,
                 const std::string& output_root_dir,
                 int mask_value_source = -1);

  void SetCameraPose(const math::RigidTransform<double>& X_WCamera,
                     systems::Context<double>* station_context) const;
  void SetObjectPose(const math::RigidTransform<double>& X_WObject,
                     systems::Context<double>* station_context) const;

  math::RigidTransform<double> GetCameraPose(
      const systems::Context<double>& context) const;
  math::RigidTransform<double> GetObjectPose(
      const systems::Context<double>& context) const;

  const multibody::MultibodyPlant<double>& plant() const { return *plant_; }

  const multibody::PoseWriter& pose_writer() const { return *pose_writer_; }

  const systems::sensors::ImageWriter& image_writer() const {
    return *image_writer_;
  }

 private:
  std::unique_ptr<multibody::MultibodyPlant<double>> owned_plant_;
  std::unique_ptr<geometry::SceneGraph<double>> owned_scene_graph_;

  multibody::MultibodyPlant<double>* plant_;
  geometry::SceneGraph<double>* scene_graph_;

  const multibody::Body<double>* camera_body_{};
  const multibody::Body<double>* object_body_{};

  multibody::PoseWriter* pose_writer_{};
  systems::sensors::ImageWriter* image_writer_{};
};

}  // namespace image_generator
}  // namespace examples
}  // namespace drake
