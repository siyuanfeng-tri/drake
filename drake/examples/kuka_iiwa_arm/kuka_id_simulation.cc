/// @file
///
/// Implements a simulation of the KUKA iiwa arm.  Like the driver for the
/// physical arm, this simulation communicates over LCM using lcmt_iiwa_status
/// and lcmt_iiwa_command messages. It is intended to be a be a direct
/// replacement for the KUKA iiwa driver and the actual robot hardware.

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/gravity_compensator.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/multiplexer.h"

#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"

#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_tree_builder.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/kuka_inverse_dynamics_servo.h"

#include "drake/examples/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"

#include "drake/examples/kuka_iiwa_arm/fake_state_estimator.h"
#include "drake/systems/analysis/explicit_euler_integrator.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::RigidBodyPlant;
using systems::Simulator;

class PickAndPlaceDemoDiagram : public systems::Diagram<double> {
 public:
  PickAndPlaceDemoDiagram(
      std::unique_ptr<RigidBodyTree<double>> world_tree,
      int iiwa_instance_id,
      int wsg_instance_id,
      int obj_instance_id,
      const std::string& iiwa_path,
      const std::string& alias_group_path,
      const std::string& controller_config_path,
      const std::string& obj_path,
      lcm::DrakeLcmInterface* lcm);

  qp_inverse_dynamics::KukaInverseDynamicsServo* get_controlled_kuka() { return iiwa_controller_; }
  IiwaCommandReceiver* get_command_receiver() { return command_receiver_; }

 private:
  systems::Multiplexer<double>* input_mux_{nullptr};
  systems::DrakeVisualizer* visualizer_{nullptr};
  qp_inverse_dynamics::KukaInverseDynamicsServo* iiwa_controller_{nullptr};
  systems::RigidBodyPlant<double>* plant_{nullptr};

  IiwaCommandReceiver* command_receiver_{nullptr};
  systems::lcm::LcmSubscriberSystem* command_sub_{nullptr};
  IiwaStatusSender* status_sender_{nullptr};
  systems::lcm::LcmPublisherSystem* status_pub_{nullptr};

  std::unique_ptr<RigidBodyTree<double>> object_{nullptr};
};

PickAndPlaceDemoDiagram::PickAndPlaceDemoDiagram(
    std::unique_ptr<RigidBodyTree<double>> world_tree,
    int iiwa_instance_id,
    int wsg_instance_id,
    int obj_instance_id,
    const std::string& iiwa_path,
    const std::string& alias_group_path,
    const std::string& controller_config_path,
    const std::string& object_path,
    lcm::DrakeLcmInterface* lcm) {
  for (int i = 0; i < world_tree->get_num_bodies(); i++) {
    std::cout << "i: " << i << ", " << world_tree->get_body(i).get_name() << std::endl;
  }

  DiagramBuilder<double> builder;

  // Makes rbp first.
  plant_ = builder.AddSystem<systems::RigidBodyPlant<double>>(std::move(world_tree));
  const RigidBodyTree<double>& tree = plant_->get_rigid_body_tree();
  //plant_->set_contact_parameters(10000., 100., 10.);
  plant_->set_contact_parameters(10000, 1, 0.8, 0.0001, 1);

  ///////////////////////////////////////////////////////////////////
  // Controller crap
  iiwa_controller_ =
      builder.AddSystem<qp_inverse_dynamics::KukaInverseDynamicsServo>(
          iiwa_path, alias_group_path, controller_config_path);

  visualizer_ = builder.AddSystem<DrakeVisualizer>(tree, lcm);
  command_sub_ = builder.AddSystem(systems::lcm::LcmSubscriberSystem::Make<lcmt_iiwa_command>("IIWA_COMMAND", lcm));
  command_receiver_ = builder.AddSystem<IiwaCommandReceiver>();
  status_pub_ = builder.AddSystem(systems::lcm::LcmPublisherSystem::Make<lcmt_iiwa_status>("IIWA_STATUS", lcm));
  status_sender_ = builder.AddSystem<IiwaStatusSender>();

  // LCM input
  builder.Connect(command_sub_->get_output_port(0),
                  command_receiver_->get_input_port(0));

  // command -> controller
  builder.Connect(command_receiver_->get_output_port(0),
                  iiwa_controller_->get_input_port_nominal_iiwa_state());

  // iiwa q, qd -> controller
  builder.Connect(plant_->model_instance_state_output_port(iiwa_instance_id),
                  iiwa_controller_->get_input_port_iiwa_state());

  // controller -> plant
  builder.Connect(iiwa_controller_->get_output_port_torque(),
                  plant_->model_instance_actuator_command_input_port(iiwa_instance_id));

  // plant -> viz
  builder.Connect(plant_->state_output_port(),
                  visualizer_->get_input_port(0));

  // status pub
  builder.Connect(plant_->model_instance_state_output_port(iiwa_instance_id),
                  status_sender_->get_state_input_port());
  builder.Connect(command_receiver_->get_output_port(0),
                  status_sender_->get_command_input_port());
  builder.Connect(status_sender_->get_output_port(0),
                  status_pub_->get_input_port(0));

  auto iiwa_state_est = builder.AddSystem(
      std::make_unique<FakeStateEstimator>(iiwa_controller_->get_robot_for_control()));
  auto iiwa_state_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<bot_core::robot_state_t>(
          "IIWA_STATE_EST", lcm));
  builder.Connect(plant_->model_instance_state_output_port(iiwa_instance_id),
                  iiwa_state_est->get_input_port_state());
  builder.Connect(iiwa_state_est->get_output_port_msg(),
                  iiwa_state_pub->get_input_port(0));

  ///////////////////////////////////////////////////////////////////
  // Objector sensor
  object_ = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(object_path,
      multibody::joints::kQuaternion, object_.get());
  auto object_state_est = builder.AddSystem(std::make_unique<FakeStateEstimator>(*object_));
  auto object_state_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<bot_core::robot_state_t>(
          "OBJECT_STATE_EST", lcm));

  builder.Connect(plant_->model_instance_state_output_port(obj_instance_id),
                  object_state_est->get_input_port_state());
  builder.Connect(object_state_est->get_output_port_msg(),
                  object_state_pub->get_input_port(0));

  ///////////////////////////////////////////////////////////////////
  // WSG CRAP
  const auto& wsg_input_port =
      plant_->model_instance_actuator_command_input_port(wsg_instance_id);
  const auto& wsg_output_port =
      plant_->model_instance_state_output_port(wsg_instance_id);

  auto wsg_command_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_schunk_wsg_command>(
          "SCHUNK_WSG_COMMAND", lcm));
  auto wsg_trajectory_generator =
      builder.AddSystem<schunk_wsg::SchunkWsgTrajectoryGenerator>(
          wsg_output_port.size(), 0);

  auto wsg_status_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_schunk_wsg_status>(
          "SCHUNK_WSG_STATUS", lcm));
  auto wsg_status_sender = builder.AddSystem<schunk_wsg::SchunkWsgStatusSender>(
      wsg_output_port.size(), 0, 1);

  const std::map<std::string, int> index_map =
      tree.computePositionNameToIndexMap();
  const int left_finger_position_index =
      index_map.at("left_finger_sliding_joint");
  const int position_index = plant_->FindInstancePositionIndexFromWorldIndex(
      wsg_instance_id, left_finger_position_index);
  const int velocity_index = position_index +
      plant_->get_num_positions(wsg_instance_id);

  Eigen::MatrixXd feedback_matrix = Eigen::MatrixXd::Zero(
      2 * plant_->get_num_actuators(wsg_instance_id),
      2 * plant_->get_num_positions(wsg_instance_id));
  feedback_matrix(0, position_index) = 1.;
  feedback_matrix(1, velocity_index) = 1.;
  std::unique_ptr<systems::MatrixGain<double>> feedback_selector =
      std::make_unique<systems::MatrixGain<double>>(feedback_matrix);

  // TODO(sam.creasey) The choice of constants below is completely
  // arbitrary and may not match the performance of the actual
  // gripper.
  const double wsg_kp = 3000.0;  // This seems very high, for some grasps
  // it's actually in the right power of
  // two.  We'll need to revisit this once
  // we're using the force command sent to
  // the gripper properly.
  const double wsg_ki = 0.0;
  const double wsg_kd = 5.0;
  const VectorX<double> wsg_v = VectorX<double>::Ones(wsg_input_port.size());
  auto wsg_pid_ports = systems::PidControlledSystem<double>::ConnectController(
      wsg_input_port, wsg_output_port, std::move(feedback_selector),
      wsg_v * wsg_kp, wsg_v * wsg_ki, wsg_v * wsg_kd,
      &builder);

  auto wsg_zero_source = builder.template AddSystem<systems::ConstantVectorSource<double>>(
      Eigen::VectorXd::Zero(1));
  builder.Connect(wsg_zero_source->get_output_port(),
      wsg_pid_ports.control_input_port);
  builder.Connect(wsg_command_sub->get_output_port(0),
                  wsg_trajectory_generator->get_command_input_port());
  builder.Connect(wsg_trajectory_generator->get_output_port(0),
                  wsg_pid_ports.state_input_port);
  builder.Connect(wsg_output_port,
                  wsg_status_sender->get_input_port(0));
  builder.Connect(wsg_output_port,
                  wsg_trajectory_generator->get_state_input_port());
  builder.Connect(*wsg_status_sender, *wsg_status_pub);

  ///////////////////////////////////////////////////////////////////


  log()->info("PickAndPlaceDemoDiagram Diagram built.");
  builder.BuildInto(this);
}

int DoMain() {
  std::string model_path =
      GetDrakePath() +
      "/examples/kuka_iiwa_arm/urdf/iiwa14_simplified_collision.urdf";
  std::string alias_group_path = GetDrakePath() +
                                 "/examples/kuka_iiwa_arm/controlled_kuka/"
                                 "inverse_dynamics_controller_config/"
                                 "kuka_alias_groups.yaml";
  std::string controller_config_path =
      GetDrakePath() +
      "/examples/kuka_iiwa_arm/controlled_kuka/"
      "inverse_dynamics_controller_config/kuka_controller.yaml";

  auto world_sim_tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();
  const std::string kRobotName =
      "/examples/kuka_iiwa_arm/urdf/"
      "iiwa14_simplified_collision.urdf";
  world_sim_tree_builder->StoreModel("iiwa", kRobotName);
  world_sim_tree_builder->StoreModel("table",
                                     "/examples/kuka_iiwa_arm/models/table/"
                                     "extra_heavy_duty_table_surface_only_"
                                     "collision.sdf");
  world_sim_tree_builder->StoreModel(
      "cylinder",
      "/examples/kuka_iiwa_arm/models/objects/simple_cuboid.urdf");
  world_sim_tree_builder->StoreModel(
      "wsg", "/examples/schunk_wsg/models/schunk_wsg_50.sdf");

  // The `z` coordinate of the top of the table in the world frame.
  // The quantity 0.736 is the `z` coordinate of the frame associated with the
  // 'surface' collision element in the SDF. This element uses a box of height
  // 0.057m thus giving the surface height (`z`) in world coordinates as
  // 0.736 + 0.057 / 2.
  const double kTableTopZInWorld = 0.736 + 0.057 / 2;
  const double kTableTopXInWorld = 0.243716;
  const double kTableTopYInWorld = 0.625087;

//  const double kTableSurfaceSizeX = 0.7112;
//  const double kTableSurfaceSizeY = 0.762;

  // robot base
  world_sim_tree_builder->AddFixedModelInstance(
      "table", Eigen::Vector3d(kTableTopXInWorld, kTableTopYInWorld, 0),
      Eigen::Vector3d::Zero());

  // start table
  world_sim_tree_builder->AddFixedModelInstance(
      "table", Eigen::Vector3d(kTableTopXInWorld + 0.8, kTableTopYInWorld, 0),
      Eigen::Vector3d::Zero());

  // end table
  world_sim_tree_builder->AddFixedModelInstance(
      "table", Eigen::Vector3d(kTableTopXInWorld, kTableTopYInWorld + 0.83, 0),
      Eigen::Vector3d::Zero());

  world_sim_tree_builder->AddGround();

  // The positions of the iiwa robot and the 3 cylinders are distributed
  // over the surface of the heavy duty table. Only the positions are set; the
  // default orientations are used in each case. While 2 of the cylinders
  // are located at table corners, the third cylinder is placed exactly
  // in between them.
  const Eigen::Vector3d kRobotBase(kTableTopXInWorld, kTableTopYInWorld,
                                   kTableTopZInWorld);

  const Eigen::Vector3d kCylinderStart(
      0.8,
      0,
      kTableTopZInWorld + 0.1);

  // Adding each model to the Tree builder.
  int iiwa_instance_id =
      world_sim_tree_builder->AddFixedModelInstance("iiwa", Vector3<double>(0, 0, kTableTopZInWorld));

  int wsg_instance_id = world_sim_tree_builder->AddModelInstanceToFrame(
      "wsg", Eigen::Vector3d::Zero(),  Eigen::Vector3d::Zero(),
      world_sim_tree_builder->tree().findFrame("iiwa_frame_ee"),
      drake::multibody::joints::kFixed);

  int object_instance_id = world_sim_tree_builder->AddFloatingModelInstance("cylinder",
                                                   kCylinderStart);

  drake::lcm::DrakeLcm lcm;
  ///////////////////////////////
  /*
  std::unique_ptr<RigidBodyTree<double>> tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
          model_path,
          drake::multibody::joints::kFixed,
          nullptr, tree.get());
  PickAndPlaceDemoDiagram system(
      std::move(tree),
      RigidBodyTreeConstants::kFirstNonWorldModelInstanceId,
      alias_group_path, controller_config_path, &lcm);
  ///////////////////////////////
  */

  std::string iiwa_w_wsg_model_path =
      GetDrakePath() +
      "/examples/kuka_iiwa_arm/urdf/iiwa14_simplified_collision_gripper_inertia.urdf";

  std::string object_model_path =
      GetDrakePath() +
      "/examples/kuka_iiwa_arm/models/objects/simple_cylinder.urdf";

  PickAndPlaceDemoDiagram system(
      world_sim_tree_builder->Build(),
      iiwa_instance_id,
      wsg_instance_id,
      object_instance_id,
      iiwa_w_wsg_model_path,
      alias_group_path,
      controller_config_path,
      object_model_path,
      &lcm);

  Simulator<double> simulator(system);
  simulator.reset_integrator<systems::RungeKutta2Integrator<double>>(system, 3e-4, simulator.get_mutable_context());
  //simulator.reset_integrator<systems::ExplicitEulerIntegrator<double>>(system, 3e-4, simulator.get_mutable_context());

  lcm.StartReceiveThread();
  simulator.Initialize();

  Context<double>* cmd_recv_context = system.GetMutableSubsystemContext(simulator.get_mutable_context(), system.get_command_receiver());
  system.get_command_receiver()->set_initial_position(cmd_recv_context,
      VectorX<double>::Zero(7));

  qp_inverse_dynamics::KukaInverseDynamicsServo* controlled_kuka = system.get_controlled_kuka();
  controlled_kuka->Initialize(system.GetMutableSubsystemContext(simulator.get_mutable_context(), controlled_kuka));

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
