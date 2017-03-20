/// @file
///
/// This demo sets up a position controlled and gravity compensated KUKA iiwa
/// robot within a simulation to follow an arbitrarily designed plan. The
/// generated plan takes the arm from the zero configuration to reach to a
/// position in space and then repeat this reaching task with a different joint
/// configuration constraint.

#include <iostream>
#include <memory>
#include <string>

#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/sim_diagram_builder.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/systems/analysis/simulator.h"

#include "drake/examples/QPInverseDynamicsForHumanoids/system/manipulator_controller.h"
#include "robotlocomotion/robot_plan_t.hpp"
#include "drake/lcmt_manipulator_plan_move_end_effector.hpp"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

int DoMain() {
  const std::string kModelPath = drake::GetDrakePath() +
    "/examples/kuka_iiwa_arm/models/iiwa14/"
    "iiwa14_simplified_collision.urdf";

  const std::string kAliasGroupsPath =
    drake::GetDrakePath() +
    "/examples/QPInverseDynamicsForHumanoids/"
    "config/iiwa.alias_groups";

  const std::string kControlConfigPath =
    drake::GetDrakePath() +
    "/examples/QPInverseDynamicsForHumanoids/"
    "config/iiwa.id_controller_config";

  drake::lcm::DrakeLcm lcm;
  SimDiagramBuilder<double> builder;
  // Adds a plant
  {
    auto tree = std::make_unique<RigidBodyTree<double>>();
    CreateTreedFromFixedModelAtPose("/examples/kuka_iiwa_arm/models/iiwa14/iiwa14_simplified_collision.urdf", tree.get());
    builder.AddPlant(std::move(tree));
  }
  builder.AddVisualizer(&lcm);

  // Adds a iiwa controller
  auto controller =
      builder.AddController<qp_inverse_dynamics::ManipulatorController>(
          RigidBodyTreeConstants::kFirstNonWorldModelInstanceId,
          kModelPath, kAliasGroupsPath, kControlConfigPath, 0.005);

  systems::DiagramBuilder<double>* base_builder = builder.get_mutable_builder();
  auto move_ee_command_sub = base_builder->AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_manipulator_plan_move_end_effector>(
          "MOVL", &lcm));

  auto move_joints_command_sub = base_builder->AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<robotlocomotion::robot_plan_t>(
          "COMMITTED_ROBOT_PLAN", &lcm));

  base_builder->Connect(move_ee_command_sub->get_output_port(0),
                        controller->get_input_port_move_end_effector_command());
  base_builder->Connect(move_joints_command_sub->get_output_port(0),
                        controller->get_input_port_move_joints_command());

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  lcm.StartReceiveThread();

  systems::Simulator<double> simulator(*diagram);
  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);

  // Initialize the controller.
  auto& alias_groups = controller->get_alias_groups();
  const RigidBodyTree<double>& robot = controller->get_robot_for_control();
  qp_inverse_dynamics::HumanoidStatus initial_status(robot, alias_groups);
  systems::Context<double>* context = simulator.get_mutable_context();
  initial_status.UpdateKinematics(context->get_time(), VectorX<double>::Zero(7), VectorX<double>::Zero(7));
  controller->Initialize(
      initial_status,
      diagram->GetMutableSubsystemContext(context, controller));

  simulator.StepTo(1000000);

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
