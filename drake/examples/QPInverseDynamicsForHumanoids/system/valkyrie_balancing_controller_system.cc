#include <iostream>
#include <memory>
#include <thread>

#include "bot_core/atlas_command_t.hpp"
#include "bot_core/robot_state_t.hpp"
#include "drake/common/drake_path.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/joint_level_controller_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/plan_eval_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/qp_controller_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/robot_state_decoder_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_value_source.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

// This is an example qp based inverse dynamics controller loop for Valkyrie
// built from the system2 blocks.
//
// The input is a lcm message of type bot_core::robot_state_t, and the output
// is another lcm message of type bot_core::atlas_command_t.
void controller_loop() {
  // Loads model.
  std::string urdf =
      drake::GetDrakePath() +
      std::string(
          "/examples/Valkyrie/urdf/urdf/"
          "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf");
  auto robot = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      urdf, multibody::joints::kRollPitchYaw, robot.get());

  systems::DiagramBuilder<double> builder;

  lcm::DrakeLcm lcm;

  RobotStateDecoderSystem* rs_msg_to_rs =
      builder.AddSystem(std::make_unique<RobotStateDecoderSystem>(*robot));
  PlanEvalSystem* plan_eval =
      builder.AddSystem(std::make_unique<PlanEvalSystem>(*robot));
  QPControllerSystem* qp_con =
      builder.AddSystem(std::make_unique<QPControllerSystem>(*robot));
  JointLevelControllerSystem* joint_con =
      builder.AddSystem<JointLevelControllerSystem>(*robot);

  auto& robot_state_subscriber = *builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<bot_core::robot_state_t>(
          "EST_ROBOT_STATE", &lcm));
  auto& atlas_command_publisher = *builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<bot_core::atlas_command_t>(
          "ROBOT_COMMAND", &lcm));

  // lcm -> rs
  builder.Connect(robot_state_subscriber.get_output_port(0),
                  rs_msg_to_rs->get_input_port_robot_state_msg());
  // rs -> qp_input
  builder.Connect(rs_msg_to_rs->get_output_port_humanoid_status(),
                  plan_eval->get_input_port_humanoid_status());
  // rs + qp_input -> qp_output
  builder.Connect(rs_msg_to_rs->get_output_port_humanoid_status(),
                  qp_con->get_input_port_humanoid_status());
  builder.Connect(plan_eval->get_output_port_qp_input(),
                  qp_con->get_input_port_qp_input());
  // rs + qp_output -> atlas_command_t
  builder.Connect(rs_msg_to_rs->get_output_port_humanoid_status(),
                  joint_con->get_input_port_humanoid_status());
  builder.Connect(qp_con->get_output_port_qp_output(),
                  joint_con->get_input_port_qp_output());
  // atlas_command_t -> lcm
  builder.Connect(joint_con->get_output_port_atlas_command(),
                  atlas_command_publisher.get_input_port(0));

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  auto context = diagram->CreateDefaultContext();
  auto output = diagram->AllocateOutput(*context);

  systems::Context<double>* plan_eval_context =
      diagram->GetMutableSubsystemContext(context.get(), plan_eval);

  // Set plan eval's desired to the initial state.
  {
    std::string alias_groups_config =
        drake::GetDrakePath() + std::string(
                                    "/examples/QPInverseDynamicsForHumanoids/"
                                    "config/alias_groups.yaml");
    param_parsers::RigidBodyTreeAliasGroups<double> alias_groups(*robot);
    alias_groups.LoadFromYAMLFile(YAML::LoadFile(alias_groups_config));
    HumanoidStatus desired_rs(*robot, alias_groups);
    /*
    desired_rs.Update(0, desired_rs.GetNominalPosition(),
                      VectorX<double>::Zero(robot->get_num_velocities()),
                      VectorX<double>::Zero(robot->actuators.size()),
                      Vector6<double>::Zero(), Vector6<double>::Zero());
                      */
    systems::State<double>* plan_eval_state =
        plan_eval_context->get_mutable_state();
    plan_eval->SetDesired(desired_rs.GetNominalPosition(), plan_eval_state);
  }

  lcm.StartReceiveThread();

  std::cout << "controller started\n";

  while (true) {
    // Call controller.
    systems::State<double>* plan_eval_state =
        plan_eval_context->get_mutable_state();
    plan_eval->DoCalcUnrestrictedUpdate(*plan_eval_context, plan_eval_state);

    // Send LCM msg.
    diagram->Publish(*context);
  }
}

}  // end namespace qp_inverse_dynamics
}  // end namespace examples
}  // end namespace drake

int main() { drake::examples::qp_inverse_dynamics::controller_loop(); }
