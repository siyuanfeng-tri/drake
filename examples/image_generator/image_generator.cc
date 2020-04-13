#include "drake/examples/image_generator/image_generator.h"
#include <experimental/filesystem>

#include "drake/geometry/render/render_engine_vtk_factory.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/sensors/rgbd_sensor.h"

namespace drake {
namespace examples {
namespace image_generator {

using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::SceneGraph;
using geometry::render::MakeRenderEngineVtk;
using geometry::render::RenderEngineVtkParams;
using math::RigidTransform;
using math::RigidTransformd;
using math::RollPitchYaw;
using math::RotationMatrix;
using multibody::Joint;
using multibody::MultibodyPlant;
using multibody::SpatialInertia;

namespace {
std::string MakeDirs(const std::string& root_dir) {
  std::string proc_dir = root_dir + "/processed";
  std::experimental::filesystem::create_directories(proc_dir + "/image_masks");
  std::experimental::filesystem::create_directories(proc_dir + "/images");
  std::experimental::filesystem::create_directories(proc_dir +
                                                    "/rendered_images");
  return proc_dir;
}

void WriteCameraIntrinsics(
    const std::string& path,
    const geometry::render::DepthCameraProperties& cam_prop) {
  std::ofstream camera_intrinsics_out(path);
  double fy = cam_prop.height / 2 / std::tan(cam_prop.fov_y / 2);
  camera_intrinsics_out << "camera_matrix:\n";
  camera_intrinsics_out << "  cols: 3\n";
  camera_intrinsics_out << "  rows: 3\n";
  camera_intrinsics_out << "  data:\n";
  camera_intrinsics_out << "  - " << fy << "\n";
  camera_intrinsics_out << "  - " << 0.0 << "\n";
  camera_intrinsics_out << "  - " << cam_prop.width / 2 << "\n";
  camera_intrinsics_out << "  - " << 0.0 << "\n";
  camera_intrinsics_out << "  - " << fy << "\n";
  camera_intrinsics_out << "  - " << cam_prop.height / 2 << "\n";
  camera_intrinsics_out << "  - " << 0.0 << "\n";
  camera_intrinsics_out << "  - " << 0.0 << "\n";
  camera_intrinsics_out << "  - " << 1.0 << "\n";
  camera_intrinsics_out << "image_height: " << cam_prop.height << "\n";
  camera_intrinsics_out << "image_width: " << cam_prop.width << "\n";
  camera_intrinsics_out.close();
}

}  // namespace

ImageGenerator::ImageGenerator(const std::string& abs_model_path,
                               const std::string& output_root_dir) {
  owned_plant_ = std::make_unique<MultibodyPlant<double>>(0);
  owned_scene_graph_ = std::make_unique<SceneGraph<double>>();

  plant_ = owned_plant_.get();
  scene_graph_ = owned_scene_graph_.get();
  plant_->RegisterAsSourceForSceneGraph(scene_graph_);
  scene_graph_->set_name("scene_graph");
  plant_->set_name("plant");

  // Add camera dummy body.
  camera_body_ =
      &plant_->AddRigidBody("camera", multibody::SpatialInertia<double>());

  // Add object.
  multibody::Parser parser(plant_);
  const auto model_index = parser.AddModelFromFile(abs_model_path);
  const auto indices = plant_->GetBodyIndices(model_index);
  // Only support single-body objects for now.
  // Note: this could be generalized fairly easily... would just want to
  // set default/random positions for the non-floating-base elements below.
  DRAKE_DEMAND(indices.size() == 1);
  object_body_ = &plant_->get_body(indices[0]);

  plant_->Finalize();

  // Build diagram.
  systems::DiagramBuilder<double> builder;

  builder.AddSystem(std::move(owned_plant_));
  builder.AddSystem(std::move(owned_scene_graph_));

  builder.Connect(
      plant_->get_geometry_poses_output_port(),
      scene_graph_->get_source_pose_port(plant_->get_source_id().value()));
  builder.Connect(scene_graph_->get_query_output_port(),
                  plant_->get_geometry_query_input_port());

  // Add camera.
  const std::string renderer_name = "vtk_renderer";
  scene_graph_->AddRenderer(renderer_name,
                            MakeRenderEngineVtk(RenderEngineVtkParams()));

  geometry::FrameId parent_body_id =
      plant_->GetBodyFrameIdOrThrow(camera_body_->index());
  const RigidTransform<double> X_PC = RigidTransform<double>::Identity();

  const double kFocalY = 645.;
  const int kHeight = 480;
  const int kWidth = 640;
  const double fov_y = std::atan(kHeight / 2. / kFocalY) * 2;
  geometry::render::DepthCameraProperties camera_properties(
      kWidth, kHeight, fov_y, renderer_name, 0.0, 65.0);

  auto camera = builder.template AddSystem<systems::sensors::RgbdSensor>(
      parent_body_id, X_PC, camera_properties);
  builder.Connect(scene_graph_->get_query_output_port(),
                  camera->query_object_input_port());

  // Add image writer
  // These numebrs doesn't really matter.
  const double record_period = 1;
  const double start_record_t = 0;
  const std::string proc_dir = MakeDirs(output_root_dir);
  WriteCameraIntrinsics(proc_dir + "/images/camera_info.yaml",
                        camera_properties);
  image_writer_ = builder.AddSystem<systems::sensors::ImageWriter>();
  const auto& rgb_port =
      image_writer_
          ->DeclareImageInputPort<systems::sensors::PixelType::kRgba8U>(
              "camera_rgb", proc_dir + "/images/{count:06}_rgb", record_period,
              start_record_t);
  builder.Connect(camera->color_image_output_port(), rgb_port);
  const auto& depth_port =
      image_writer_
          ->DeclareImageInputPort<systems::sensors::PixelType::kDepth16U>(
              "camera_depth", proc_dir + "/rendered_images/{count:06}_depth",
              record_period, start_record_t);
  builder.Connect(camera->depth_image_16U_output_port(), depth_port);
  auto converter = builder.AddSystem<systems::sensors::PotatoMaskConverter>(
      10, kWidth, kHeight);
  const auto& label_port =
      image_writer_
          ->DeclareImageInputPort<systems::sensors::PixelType::kGrey8U>(
              "camera_label", proc_dir + "/image_masks/{count:06}_mask",
              record_period, start_record_t);
  builder.Connect(camera->label_image_output_port(),
                  converter->get_input_port(0));
  builder.Connect(converter->get_output_port(0), label_port);

  // Add pose writer.
  std::vector<const multibody::Frame<double>*> frames_to_write;
  frames_to_write.push_back(&camera_body_->body_frame());
  pose_writer_ = builder.template AddSystem<multibody::PoseWriter>(
      plant_, frames_to_write, proc_dir + "/images/pose_data.yaml",
      record_period, start_record_t);
  builder.Connect(plant_->get_state_output_port(),
                  pose_writer_->get_input_port(0));

  builder.BuildInto(this);
}

void ImageGenerator::SetCameraPose(
    const math::RigidTransform<double>& X_WCamera,
    systems::Context<double>* context) const {
  auto& plant_context = this->GetMutableSubsystemContext(*plant_, context);
  plant_->SetFreeBodyPose(&plant_context, *camera_body_, X_WCamera);
}

void ImageGenerator::SetObjectPose(
    const math::RigidTransform<double>& X_WObject,
    systems::Context<double>* context) const {
  auto& plant_context = this->GetMutableSubsystemContext(*plant_, context);
  plant_->SetFreeBodyPose(&plant_context, *object_body_, X_WObject);
}

math::RigidTransform<double> ImageGenerator::GetCameraPose(
    const systems::Context<double>& context) const {
  const auto& plant_context = this->GetSubsystemContext(*plant_, context);
  return plant_->CalcRelativeTransform(plant_context, plant_->world_frame(),
                                       camera_body_->body_frame());
}

math::RigidTransform<double> ImageGenerator::GetObjectPose(
    const systems::Context<double>& context) const {
  const auto& plant_context = this->GetSubsystemContext(*plant_, context);
  return plant_->CalcRelativeTransform(plant_context, plant_->world_frame(),
                                       object_body_->body_frame());
}

}  // namespace image_generator
}  // namespace examples
}  // namespace drake
