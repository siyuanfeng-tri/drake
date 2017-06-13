#include <iostream>
#include <memory>
#include <string>

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/sim_diagram_builder.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/manipulation/util/motion_plan_translator.h"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/lcmt_contact_results_for_viz.hpp"

#include "drake/examples/QPInverseDynamicsForHumanoids/system/iiwa_controller.h"
#include "drake/examples/kuka_iiwa_arm/oracular_state_estimator.h"
#include "drake/lcmt_motion_plan.hpp"
#include "drake/lcmt_plan_eval_debug_info.hpp"

#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_tree_builder.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

std::unique_ptr<RigidBodyTree<double>> make_rbt(
    ModelInstanceInfo<double>* iiwa_instance,
    ModelInstanceInfo<double>* box_instance) {
  const std::string kModelPath =
    "/manipulation/models/iiwa_description/urdf/"
    "sfeng_dual_iiwa14_polytope_collision.urdf";
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  tree_builder->StoreModel("iiwas", kModelPath);
  tree_builder->StoreModel("box",
      "/examples/kuka_iiwa_arm/models/objects/sfeng_box.urdf");

  tree_builder->AddGround();

  int id = tree_builder->AddFixedModelInstance("iiwas", Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  tree_builder->get_model_info_for_instance(id);
  *iiwa_instance = tree_builder->get_model_info_for_instance(id);;

  id = tree_builder->AddFloatingModelInstance("box",
                                              Vector3<double>(0.54 + 0.1, 0, 0.95 + 0.1),
                                              Vector3<double>::Zero());
  *box_instance = tree_builder->get_model_info_for_instance(id);

  auto tree = tree_builder->Build();

  Isometry3<double> X_WStand = Isometry3<double>::Identity();
  X_WStand.translation() = Vector3<double>(0.54 + 0.1, 0, 0.95 - 0.1 - 0.15);
  multibody::AddBoxToWorld(X_WStand, Vector3<double>(0.3, 0.3, 0.3), "stand", tree.get());

  return tree;
}

int DoMain() {
  const std::string kAliasGroupsPath =
    drake::GetDrakePath() +
    "/examples/QPInverseDynamicsForHumanoids/"
    "config/sfeng_iiwa.alias_groups";

  const std::string kControlConfigPath =
    drake::GetDrakePath() +
    "/examples/QPInverseDynamicsForHumanoids/"
    "config/sfeng_iiwa.id_controller_config";
  drake::lcm::DrakeLcm lcm;
  SimDiagramBuilder<double> builder;
  systems::RigidBodyPlant<double>* plant;

  ModelInstanceInfo<double> iiwa_instance;
  ModelInstanceInfo<double> box_instance;
  {
    auto tree = make_rbt(&iiwa_instance, &box_instance);

    // Adds a plant
    plant = builder.AddPlant(std::move(tree));
  }

  auto viz = builder.AddVisualizer(&lcm);
  viz->set_publish_period(1e-2);

  // Adds a iiwa controller
  auto controller =
      builder.AddController<qp_inverse_dynamics::IiwaController>(
          iiwa_instance.instance_id,
          iiwa_instance.model_path,
          kAliasGroupsPath, kControlConfigPath, 1e-3, nullptr,
          box_instance.model_path,
          box_instance.world_offset);
  const RigidBodyTree<double>& robot = controller->get_robot_for_control();

  systems::DiagramBuilder<double>* diagram_builder =
      builder.get_mutable_builder();

  auto plan_sub = diagram_builder->AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_motion_plan>(
          "IIWA_PLAN", &lcm));

  auto contact_viz =
      diagram_builder->AddSystem<systems::ContactResultsToLcmSystem<double>>(
          plant->get_rigid_body_tree());
  contact_viz->set_name("contact_viz");

  auto contact_results_publisher = diagram_builder->AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", &lcm));
  contact_results_publisher->set_name("contact_results_publisher");

  diagram_builder->Connect(plant->contact_results_output_port(),
      contact_viz->get_input_port(0));
  diagram_builder->Connect(contact_viz->get_output_port(0),
      contact_results_publisher->get_input_port(0));

  contact_results_publisher->set_publish_period(1e-2);

  // Plan receiver -> controller.
  diagram_builder->Connect(plan_sub->get_output_port(0),
                           controller->get_input_port_plan());

  std::cout << plant->model_instance_state_output_port(box_instance.instance_id).size() << ", " <<
      controller->get_input_port_object_state().size() << std::endl;
  diagram_builder->Connect(plant->model_instance_state_output_port(box_instance.instance_id),
                           controller->get_input_port_object_state());

  auto estimator = diagram_builder->AddSystem<OracularStateEstimation<double>>(
      robot);
  diagram_builder->Connect(
      plant->model_instance_state_output_port(iiwa_instance.instance_id),
      estimator->get_input_port_state());

  // EST_ROBOT_STATE -> publisher.
  auto iiwa_state_pub = diagram_builder->AddSystem(
      systems::lcm::LcmPublisherSystem::Make<bot_core::robot_state_t>(
          "IIWA_STATE_EST", &lcm));
  diagram_builder->Connect(estimator->get_output_port_msg(),
                           iiwa_state_pub->get_input_port(0));

  iiwa_state_pub->set_publish_period(1e-2);

  auto plan_eval_pub = diagram_builder->AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_plan_eval_debug_info>(
          "PLAN_EVAL", &lcm));
  diagram_builder->Connect(controller->get_output_port_debug_info(),
                           plan_eval_pub->get_input_port(0));
  plan_eval_pub->set_publish_period(1e-2);

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);

  // TODO set real init.
  systems::Context<double>* controller_context = diagram->GetMutableSubsystemContext(
      simulator.get_mutable_context(), controller);
  VectorX<double> zero = VectorX<double>::Zero(robot.get_num_positions());
  controller->Initialize(0, zero, zero, controller_context);

  lcm.StartReceiveThread();

  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);
  simulator.get_mutable_integrator()->set_maximum_step_size(1e-4);
  simulator.set_publish_every_time_step(false);

  simulator.StepTo(INFINITY);

  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::kuka_iiwa_arm::DoMain();
}
