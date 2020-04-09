#include "drake/examples/manipulation_station/pdc_common.h"

#include <experimental/filesystem>

#include "drake/common/find_resource.h"
#include "drake/examples/manipulation_station/manipulation_station.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"
#include "drake/systems/sensors/image_writer.h"
#include "drake/systems/sensors/mask_converter.h"

#include "drake/examples/manipulation_station/point_cloud_visualizer.h"
#include "drake/examples/manipulation_station/pose_writer.h"
#include "drake/manipulation/robot_bridge/robot_bridge.h"
#include "drake/multibody/plant/frame_visualizer.h"
#include "drake/multibody/plant/kinematics_calculator.h"
#include "drake/systems/sensors/rgbd_sensor.h"

namespace drake {
namespace examples {
namespace manipulation_station {
namespace {

using Eigen::VectorXd;
using math::RigidTransform;
using math::RollPitchYaw;
using math::RotationMatrix;

template <typename T>
multibody::ModelInstanceIndex AddAndWeldModelFrom(
    const std::string& model_path, const std::string& model_name,
    const multibody::Frame<T>& parent, const std::string& child_frame_name,
    const RigidTransform<double>& X_PC, multibody::MultibodyPlant<T>* plant) {
  DRAKE_THROW_UNLESS(!plant->HasModelInstanceNamed(model_name));

  multibody::Parser parser(plant);
  const multibody::ModelInstanceIndex new_model =
      parser.AddModelFromFile(model_path, model_name);
  const auto& child_frame = plant->GetFrameByName(child_frame_name, new_model);
  plant->WeldFrames(parent, child_frame, X_PC);
  return new_model;
}

std::string MakeDirs(const std::string& root_dir) {
  // std::string mk_dir = GetExperimentRootDir();
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

MegaDiagramStuff build_pdc_mega_diagram(
    const std::string& obj_name, const std::string& model_dir,
    bool visualize_point_cloud,
    const std::optional<DataRecordParams>& record_params,
    const std::optional<PeelCommandParams>& peel_cmd_params) {
  systems::DiagramBuilder<double> builder;

  // Make a robot bridge for control.
  auto rb_plant = std::make_unique<multibody::MultibodyPlant<double>>(0.0);
  std::string iiwa_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/iiwa7/"
      "iiwa7_no_collision_w_slide.sdf");
  AddAndWeldModelFrom(iiwa_path, "iiwa", rb_plant->world_frame(), "shim_0",
                      RigidTransform<double>::Identity(), rb_plant.get());
  auto X_7T = math::RigidTransform<double>(
      math::RollPitchYaw<double>(0, M_PI, 0), Eigen::Vector3d(0, 0, 0.2));
  const auto& link7 = rb_plant->GetFrameByName("iiwa_link_7");
  rb_plant->AddFrame(std::make_unique<multibody::FixedOffsetFrame<double>>(
      "tool_frame", link7, X_7T));
  rb_plant->Finalize();

  // Create the "manipulation station".
  auto station = builder.AddSystem<ManipulationStation>();
  math::RigidTransform<double> X_WC(
      math::RollPitchYaw<double>(-0 * M_PI / 180., 0, M_PI_2),
      Eigen::Vector3d(0.0, 0, 0.114));
  station->SetupPdcDataCollectStation(X_WC);

  // Add manipuland.
  std::string model_file;
  if (model_dir.empty()) {
    model_file =
        "drake/manipulation/models/" + obj_name + "/" + obj_name + ".sdf";
    station->AddManipulandFromFile(model_file, math::RigidTransform<double>());
  } else {
    model_file = model_dir;
    if (model_file.back() != '/') model_file += "/";
    model_file += obj_name + "/" + obj_name + ".sdf";
    station->AddManipulandFromAbsoluteFile(model_file,
                                           math::RigidTransform<double>());
  }

  station->SetIiwaPositionGains(
      Eigen::VectorXd::Constant(rb_plant->num_positions(), 1e3));
  station->Finalize();

  geometry::ConnectDrakeVisualizer(&builder, station->get_mutable_scene_graph(),
                                   station->GetOutputPort("pose_bundle"));
  multibody::ConnectContactResultsToDrakeVisualizer(
      &builder, station->get_mutable_multibody_plant(),
      station->GetOutputPort("contact_results"));

  // Image publisher over lcm, stuff. (These aren't saved to disk, just for viz)
  auto image_to_lcm_image_array =
      builder.template AddSystem<systems::sensors::ImageToLcmImageArrayT>();
  image_to_lcm_image_array->set_name("converter");
  for (const auto& name : station->get_camera_names()) {
    drake::log()->info("camera name: {}", name);
    const auto& rgb_port =
        image_to_lcm_image_array
            ->DeclareImageInputPort<systems::sensors::PixelType::kRgba8U>(
                "camera_" + name + "_rgb");
    builder.Connect(station->GetOutputPort("camera_" + name + "_rgb_image"),
                    rgb_port);
    const auto& depth_port =
        image_to_lcm_image_array
            ->DeclareImageInputPort<systems::sensors::PixelType::kDepth16U>(
                "camera_" + name + "_depth");
    builder.Connect(station->GetOutputPort("camera_" + name + "_depth_image"),
                    depth_port);
    const auto& label_port =
        image_to_lcm_image_array
            ->DeclareImageInputPort<systems::sensors::PixelType::kLabel16I>(
                "camera_" + name + "_label");
    builder.Connect(station->GetOutputPort("camera_" + name + "_label_image"),
                    label_port);
  }
  auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();

  const double kCameraFps = 10.;
  auto image_array_lcm_publisher = builder.template AddSystem(
      systems::lcm::LcmPublisherSystem::Make<robotlocomotion::image_array_t>(
          "DRAKE_RGBD_CAMERA_IMAGES", lcm, 1.0 / kCameraFps));

  image_array_lcm_publisher->set_name("rgbd_publisher");
  builder.Connect(image_to_lcm_image_array->image_array_t_msg_output_port(),
                  image_array_lcm_publisher->get_input_port());

  const auto& plant = station->get_multibody_plant();
  // Frame Visualizer (only for viz)
  auto kin_calc =
      builder.template AddSystem<multibody::KinematicsCaculator>(&plant);
  builder.Connect(station->GetOutputPort("plant_continuous_state"),
                  kin_calc->get_input_port(0));

  std::vector<const multibody::Frame<double>*> frames_to_viz;
  frames_to_viz.push_back(&plant.GetFrameByName("wrist_camera_frame"));
  frames_to_viz.push_back(&plant.GetFrameByName("iiwa_link_7"));
  frames_to_viz.push_back(&plant.GetFrameByName(obj_name));
  frames_to_viz.push_back(&plant.GetFrameByName("iiwa_link_0"));
  frames_to_viz.push_back(&plant.GetFrameByName("tool_frame"));
  {
    auto frame_viz = builder.template AddSystem<multibody::FrameVisualizer>(
        &plant, frames_to_viz);
    auto frame_viz_lcm_publisher = builder.template AddSystem(
        systems::lcm::LcmPublisherSystem::Make<lcmt_viewer_draw>(
            "DRAKE_DRAW_FRAMES", lcm, 1 / 60.));
    frame_viz_lcm_publisher->set_name("frame_viz_lcm_publisher");
    builder.Connect(kin_calc->get_output_port(0), frame_viz->get_input_port(0));
    builder.Connect(frame_viz->get_output_port(0),
                    frame_viz_lcm_publisher->get_input_port());
  }

  // Hack camera odometry.
  {
    auto frame_viz = builder.template AddSystem<multibody::FrameVisualizer>(
        &plant, frames_to_viz);
    frame_viz->set_name("ha");
    auto frame_viz_lcm_publisher = builder.template AddSystem(
        systems::lcm::LcmPublisherSystem::Make<lcmt_viewer_draw>(
            "X_WC_ODOMETRY", lcm, 1. / kCameraFps));
    frame_viz_lcm_publisher->set_name("X_WC_odometry publisher");
    builder.Connect(kin_calc->get_output_port(0), frame_viz->get_input_port(0));
    builder.Connect(frame_viz->get_output_port(0),
                    frame_viz_lcm_publisher->get_input_port());
  }

  const auto& tool_frame = rb_plant->GetFrameByName("tool_frame");
  auto robot_comm = builder.AddSystem<manipulation::robot_bridge::RobotBridge>(
      rb_plant.get(), &tool_frame, 1e-3);

  builder.Connect(station->GetOutputPort("iiwa_state_estimated"),
                  robot_comm->GetInputPort("state"));
  builder.Connect(robot_comm->GetOutputPort("position"),
                  station->GetInputPort("iiwa_position"));
  builder.Connect(robot_comm->GetOutputPort("torque"),
                  station->GetInputPort("iiwa_feedforward_torque"));

  /////////////////////////////////////////////////////////////////////////////
  /// Optional stuff.
  /////////////////////////////////////////////////////////////////////////////
  // Point cloud vis.
  if (visualize_point_cloud) {
    for (const auto& name : station->get_camera_names()) {
      const auto& prop = station->GetCameraProperties(name);
      auto point_cloud_viz = builder.template AddSystem<PointCloudVisualizer>(
          prop, &plant, &plant.GetFrameByName("wrist_camera_frame"));
      builder.Connect(station->GetOutputPort("camera_" + name + "_depth_image"),
                      point_cloud_viz->get_depth_image_input());
      builder.Connect(station->GetOutputPort("camera_" + name + "_rgb_image"),
                      point_cloud_viz->get_rgb_image_input());
      builder.Connect(kin_calc->get_output_port(0),
                      point_cloud_viz->get_kinematics_input());
      auto pub = builder.template AddSystem(
          systems::lcm::LcmPublisherSystem::Make<bot_core::pointcloud_t>(
              "DRAKE_POINTCLOUD_RGBD", lcm, 1.0 / kCameraFps));
      builder.Connect(point_cloud_viz->get_output_port(0),
                      pub->get_input_port());
    }
  }

  // Make directories. (need to happen before making image writer)
  const bool is_data_gen = record_params.has_value();
  if (is_data_gen) {
    const std::string proc_dir = MakeDirs(record_params->record_root_dir);
    // Write camera intrinsics
    const geometry::render::DepthCameraProperties& cam_prop =
        station->GetCameraProperties("wrist_camera");
    WriteCameraIntrinsics(proc_dir + "/images/camera_info.yaml", cam_prop);

    // Image writer.
    auto image_writer = builder.AddSystem<systems::sensors::ImageWriter>();
    const double start_record_t = record_params->record_start_time;
    const double record_period = record_params->record_period;
    for (const auto& name : station->get_camera_names()) {
      const auto& rgb_port =
          image_writer
              ->DeclareImageInputPort<systems::sensors::PixelType::kRgba8U>(
                  "camera_" + name + "_rgb",
                  proc_dir + "/images/{count:06}_rgb", record_period,
                  start_record_t);
      builder.Connect(station->GetOutputPort("camera_" + name + "_rgb_image"),
                      rgb_port);
      const auto& depth_port =
          image_writer
              ->DeclareImageInputPort<systems::sensors::PixelType::kDepth16U>(
                  "camera_" + name + "_depth",
                  proc_dir + "/rendered_images/{count:06}_depth", record_period,
                  start_record_t);
      builder.Connect(station->GetOutputPort("camera_" + name + "_depth_image"),
                      depth_port);

      static_cast<int>(plant.GetBodyByName(obj_name).index());
      // everything else is setup correctly except the horizon, which will be
      // 255. so we have a filtering hack to get rid of it.
      auto converter = builder.AddSystem<systems::sensors::PotatoMaskConverter>(
          10, cam_prop.width, cam_prop.height);
      const auto& label_port =
          image_writer
              ->DeclareImageInputPort<systems::sensors::PixelType::kGrey8U>(
                  "camera_" + name + "_label",
                  proc_dir + "/image_masks/{count:06}_mask", record_period,
                  start_record_t);
      builder.Connect(station->GetOutputPort("camera_" + name + "_label_image"),
                      converter->get_input_port(0));
      builder.Connect(converter->get_output_port(0), label_port);
    }

    // Pose writer.
    std::vector<const multibody::Frame<double>*> frames_to_write;
    frames_to_write.push_back(&plant.GetFrameByName("wrist_camera_frame"));
    auto pose_writer = builder.template AddSystem<multibody::PoseWriter>(
        &plant, frames_to_write, proc_dir + "/images/pose_data.yaml",
        record_period, start_record_t);
    builder.Connect(station->GetOutputPort("plant_continuous_state"),
                    pose_writer->get_input_port(0));
  }

  systems::lcm::LcmSubscriberSystem* peel_cmd_sub{};
  if (peel_cmd_params.has_value()) {
    peel_cmd_sub = builder.AddSystem(
        systems::lcm::LcmSubscriberSystem::Make<lcmt_viewer_draw>(
            peel_cmd_params->channel_name, lcm));
  }

  auto diagram = builder.Build();

  return {std::move(diagram), std::move(rb_plant), station, robot_comm,
          peel_cmd_sub};
}

RigidTransform<double> CalcRelativeTransform(
    const multibody::MultibodyPlant<double>& plant, const Eigen::VectorXd& x,
    const multibody::Frame<double>& frame_A,
    const multibody::Frame<double>& frame_B) {
  auto context = plant.CreateDefaultContext();
  plant.SetPositionsAndVelocities(context.get(), x);
  return plant.CalcRelativeTransform(*context, frame_A, frame_B);
}

}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake
