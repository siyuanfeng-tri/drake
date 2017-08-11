/// @file
///
/// Implements a simulation of the KUKA iiwa arm.  Like the driver for the
/// physical arm, this simulation communicates over LCM using lcmt_iiwa_status
/// and lcmt_iiwa_command messages. It is intended to be a be a direct
/// replacement for the KUKA iiwa driver and the actual robot hardware.

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/frame_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/adder.h"

#include "drake/examples/kuka_iiwa_arm/jjz_common.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"

#ifdef CAMERA
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"
#include "drake/systems/sensors/rgbd_camera.h"

constexpr char kColorCameraFrameName[] = "color_camera_optical_frame";
constexpr char kDepthCameraFrameName[] = "depth_camera_optical_frame";
constexpr char kLabelCameraFrameName[] = "label_camera_optical_frame";
constexpr char kImageArrayLcmChannelName[] = "DRAKE_RGBD_CAMERA_IMAGES";
#endif

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");
DEFINE_string(urdf, "", "Name of urdf to load");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

using systems::ConstantVectorSource;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::FrameVisualizer;
using systems::RigidBodyPlant;
using systems::Simulator;

int DoMain() {
  drake::lcm::DrakeLcm lcm;
  DiagramBuilder<double> builder;

  // Adds a plant.
  RigidBodyPlant<double>* plant = nullptr;
  const char* kModelPath =
      "drake/manipulation/models/iiwa_description/"
      "urdf/iiwa14_polytope_collision.urdf";
  const std::string urdf =
      (!FLAGS_urdf.empty() ? FLAGS_urdf : FindResourceOrThrow(kModelPath));
  {
    auto tree = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        urdf, multibody::joints::kFixed, tree.get());
    multibody::AddFlatTerrainToWorld(tree.get(), 100., 10.);
    plant = builder.AddSystem<systems::RigidBodyPlant<double>>(std::move(tree));
    plant->set_name("name");
  }
  // Creates and adds LCM publisher for visualization.
  auto viz = builder.AddSystem<systems::DrakeVisualizer>(
      plant->get_rigid_body_tree(), &lcm);
  viz->set_name("visualizer");
  viz->set_publish_period(kIiwaLcmStatusPeriod);

  const RigidBodyTree<double>& tree = plant->get_rigid_body_tree();
  const int num_joints = tree.get_num_positions();

  // Adds a iiwa controller
  VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
  SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);

  DRAKE_DEMAND(tree.get_num_positions() % kIiwaArmNumJoints == 0);
  for (int offset = kIiwaArmNumJoints; offset < tree.get_num_positions();
       offset += kIiwaArmNumJoints) {
    const int end = offset + kIiwaArmNumJoints;
    iiwa_kp.conservativeResize(end);
    iiwa_kp.segment(offset, kIiwaArmNumJoints) =
        iiwa_kp.head(kIiwaArmNumJoints);
    iiwa_ki.conservativeResize(end);
    iiwa_ki.segment(offset, kIiwaArmNumJoints) =
        iiwa_ki.head(kIiwaArmNumJoints);
    iiwa_kd.conservativeResize(end);
    iiwa_kd.segment(offset, kIiwaArmNumJoints) =
        iiwa_kd.head(kIiwaArmNumJoints);
  }

  auto controller =
      builder
          .AddSystem<systems::controllers::InverseDynamicsController<double>>(
              tree.Clone(), iiwa_kp, iiwa_ki, iiwa_kd,
              false /* without feedforward acceleration */);

  auto adder = builder.AddSystem<systems::Adder<double>>(2, 7);

  // Create the command subscriber and status publisher.
  auto command_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_iiwa_command>("IIWA_COMMAND",
                                                                 &lcm));
  command_sub->set_name("command_subscriber");
  auto command_receiver = builder.AddSystem<IiwaCommandReceiver>(num_joints);
  command_receiver->set_name("command_receiver");
  auto status_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_iiwa_status>("IIWA_STATUS",
                                                               &lcm));
  status_pub->set_name("status_publisher");
  status_pub->set_publish_period(kIiwaLcmStatusPeriod);
  auto status_sender = builder.AddSystem<IiwaStatusSender>(&tree);
  // auto status_sender = builder.AddSystem<IiwaStatusSender>(num_joints);
  status_sender->set_name("status_sender");

//////////////////////////////////////////////////////////////////////
// Add a rgbd camera
#ifdef CAMERA
  Isometry3<double> X_BC =
      Eigen::Translation3d(0., 0.02, 0.) *
      Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());

  Isometry3<double> X_7C = Isometry3<double>::Identity();
  X_7C.translation() = Vector3<double>(0.1, 0, 0.1);
  X_7C.linear() = Eigen::AngleAxisd(-M_PI / 4., Eigen::Vector3d::UnitY())
                      .toRotationMatrix();
  RigidBodyFrame<double> F_7B("X_7B", tree.FindBody("iiwa_link_7"),
                              X_7C * X_BC.inverse());
  auto rgbd = builder.AddSystem<systems::sensors::RgbdCamera>(
      "rgbd", tree, F_7B, 45. * M_PI / 180., true);
  builder.Connect(plant->get_output_port(0), rgbd->state_input_port());
  auto image_to_lcm_image_array =
      builder.AddSystem<systems::sensors::ImageToLcmImageArrayT>(
          kColorCameraFrameName, kDepthCameraFrameName, kLabelCameraFrameName);
  image_to_lcm_image_array->set_name("converter");
  auto image_array_lcm_publisher = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<robotlocomotion::image_array_t>(
          kImageArrayLcmChannelName, &lcm));
  image_array_lcm_publisher->set_name("publisher");
  image_array_lcm_publisher->set_publish_period(1);
  builder.Connect(rgbd->color_image_output_port(),
                  image_to_lcm_image_array->color_image_input_port());
  builder.Connect(rgbd->depth_image_output_port(),
                  image_to_lcm_image_array->depth_image_input_port());
  builder.Connect(rgbd->label_image_output_port(),
                  image_to_lcm_image_array->label_image_input_port());
  builder.Connect(image_to_lcm_image_array->image_array_t_msg_output_port(),
                  image_array_lcm_publisher->get_input_port(0));
#endif
  //////////////////////////////////////////////////////////////////////
  auto contact_viz =
    builder.AddSystem<systems::ContactResultsToLcmSystem<double>>(
        tree);
  auto contact_results_publisher = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
        "CONTACT_RESULTS", &lcm));
  // Contact results to lcm msg.
  builder.Connect(plant->contact_results_output_port(),
      contact_viz->get_input_port(0));
  builder.Connect(contact_viz->get_output_port(0),
      contact_results_publisher->get_input_port(0));
  contact_results_publisher->set_publish_period(kIiwaLcmStatusPeriod);

  builder.Connect(plant->get_output_port(0),
                  controller->get_input_port_estimated_state());
  builder.Connect(controller->get_output_port_control(),
                  adder->get_input_port(0));
  builder.Connect(command_receiver->get_output_port(1),
                  adder->get_input_port(1));

  builder.Connect(adder->get_output_port(),
                  plant->actuator_command_input_port());
  builder.Connect(plant->get_output_port(0), viz->get_input_port(0));

  builder.Connect(command_sub->get_output_port(0),
                  command_receiver->get_input_port(0));
  builder.Connect(command_receiver->get_output_port(0),
                  controller->get_input_port_desired_state());
  builder.Connect(plant->get_output_port(0),
                  status_sender->get_state_input_port());
  builder.Connect(command_receiver->get_output_port(0),
                  status_sender->get_command_input_port());

  builder.Connect(adder->get_output_port(),
                  status_sender->get_torque_input_port());
  builder.Connect(status_sender->get_output_port(0),
                  status_pub->get_input_port(0));

  // Visualizes the end effector frame and 7th body's frame.
  std::vector<RigidBodyFrame<double>> local_transforms;
  local_transforms.push_back(RigidBodyFrame<double>(
      "iiwa_tool", tree.FindBody(jjz::kEEName), jjz::X_ET));
  local_transforms.push_back(
      RigidBodyFrame<double>("goal", tree.FindBody("world"), jjz::X_WG));
  Isometry3<double> X_WTag(Eigen::Translation<double, 3>(Vector3<double>(1.5, -0.3, 0.2)));
  local_transforms.push_back(
      RigidBodyFrame<double>("AprilTag", tree.FindBody("world"), X_WTag));
#ifdef CAMERA
  local_transforms.push_back(
      RigidBodyFrame<double>("camera", tree.FindBody("iiwa_link_7"), X_7C));
#endif
  auto frame_viz = builder.AddSystem<systems::FrameVisualizer>(
      &tree, local_transforms, &lcm);
  builder.Connect(plant->get_output_port(0), frame_viz->get_input_port(0));
  frame_viz->set_publish_period(kIiwaLcmStatusPeriod);

  auto sys = builder.Build();

  Simulator<double> simulator(*sys);

  lcm.StartReceiveThread();
  simulator.set_publish_every_time_step(false);
  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);

  VectorX<double> q0 = tree.getZeroConfiguration();

  Context<double>& plant_context =
      sys->GetMutableSubsystemContext(*plant, simulator.get_mutable_context());
  systems::VectorBase<double>* plant_q0 =
      plant_context.get_mutable_continuous_state()
          ->get_mutable_generalized_position();
  plant_q0->SetFromVector(q0);

  command_receiver->set_initial_position(
      &sys->GetMutableSubsystemContext(*command_receiver,
                                       simulator.get_mutable_context()),
      q0);

  // Simulate for a very long time.
  simulator.StepTo(FLAGS_simulation_sec);

  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kuka_iiwa_arm::DoMain();
}
