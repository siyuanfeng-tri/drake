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

class IdControlledPlantWithRobot : public systems::Diagram<double> {
 public:
  IdControlledPlantWithRobot(
      std::unique_ptr<RigidBodyTree<double>> world_tree,
      int robot_model_instance_id,
      const std::string& alias_group_path,
      const std::string& controller_config_path,
      lcm::DrakeLcmInterface* lcm);

  qp_inverse_dynamics::KukaInverseDynamicsServo* get_controlled_kuka() { return controlled_kuka_; }
  IiwaCommandReceiver* get_command_receiver() { return command_receiver_; }

 private:
  systems::Multiplexer<double>* input_mux_{nullptr};
  systems::DrakeVisualizer* visualizer_{nullptr};
  qp_inverse_dynamics::KukaInverseDynamicsServo* controlled_kuka_{nullptr};

  IiwaCommandReceiver* command_receiver_{nullptr};
  systems::lcm::LcmSubscriberSystem* command_sub_{nullptr};
  IiwaStatusSender* status_sender_{nullptr};
  systems::lcm::LcmPublisherSystem* status_pub_{nullptr};

};

IdControlledPlantWithRobot::IdControlledPlantWithRobot(
    std::unique_ptr<RigidBodyTree<double>> world_tree,
    int robot_instance_id,
    const std::string& alias_group_path,
    const std::string& controller_config_path,
    lcm::DrakeLcmInterface* lcm) {
  DiagramBuilder<double> builder;

  controlled_kuka_ =
      builder.template AddSystem<qp_inverse_dynamics::KukaInverseDynamicsServo>(
          std::move(world_tree), robot_instance_id,
          alias_group_path, controller_config_path);

  const RigidBodyPlant<double>& plant = controlled_kuka_->get_plant();
  const RigidBodyTree<double>& tree = plant.get_rigid_body_tree();

  visualizer_ = builder.AddSystem<DrakeVisualizer>(tree, lcm);
  command_sub_ = builder.AddSystem(systems::lcm::LcmSubscriberSystem::Make<lcmt_iiwa_command>("IIWA_COMMAND", lcm));
  command_receiver_ = builder.AddSystem<IiwaCommandReceiver>();
  status_pub_ = builder.AddSystem(systems::lcm::LcmPublisherSystem::Make<lcmt_iiwa_status>("IIWA_STATUS", lcm));
  status_sender_ = builder.AddSystem<IiwaStatusSender>();

  builder.Connect(command_sub_->get_output_port(0),
                  command_receiver_->get_input_port(0));

  builder.Connect(command_receiver_->get_output_port(0),
                  controlled_kuka_->get_input_port(0));

  builder.Connect(controlled_kuka_->get_output_port(0),
                  visualizer_->get_input_port(0));

  builder.Connect(controlled_kuka_->get_output_port(1),
                  status_sender_->get_state_input_port());

  builder.Connect(command_receiver_->get_output_port(0),
                  status_sender_->get_command_input_port());

  builder.Connect(status_sender_->get_output_port(0),
                  status_pub_->get_input_port(0));

  log()->info("IdControlledPlantWithRobot Diagram built.");
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
      "/examples/kuka_iiwa_arm/models/objects/simple_cylinder.urdf");

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


//  const double kTableSurfaceSizeX = 0.7112;
//  const double kTableSurfaceSizeY = 0.762;
//  const double kGapToEdge = 0.05;

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

  const Eigen::Vector3d kCylinderEnd(
      0,
      0.83,
      kTableTopZInWorld + 0.1);

  std::cout << "start " << kCylinderStart.transpose() << std::endl;
  std::cout << "end " << kCylinderEnd.transpose() << std::endl;

  // Adding each model to the Tree builder.
  int robot_model_instance =
      world_sim_tree_builder->AddFixedModelInstance("iiwa", Vector3<double>(0, 0, kTableTopZInWorld));
  std::cout << robot_model_instance << std::endl;

  world_sim_tree_builder->AddFloatingModelInstance("cylinder",
                                                   kCylinderStart);

  world_sim_tree_builder->AddFloatingModelInstance("cylinder",
                                                   kCylinderEnd);

  drake::lcm::DrakeLcm lcm;

  ///////////////////////////////
  /*
  std::unique_ptr<RigidBodyTree<double>> tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
          model_path,
          drake::multibody::joints::kFixed,
          nullptr, tree.get());
  IdControlledPlantWithRobot system(
      std::move(tree),
      RigidBodyTreeConstants::kFirstNonWorldModelInstanceId,
      alias_group_path, controller_config_path, &lcm);
  ///////////////////////////////
  */

  IdControlledPlantWithRobot system(
      world_sim_tree_builder->Build(),
      robot_model_instance,
      alias_group_path, controller_config_path, &lcm);

  Simulator<double> simulator(system);

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
