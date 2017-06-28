/// @file
///
/// This demo sets up a simple passive dynamics simulation of the Fetch mobile
/// robot, i.e., all joint torques are set to zero.

#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/Fetch/fetch_common.h"
#include "drake/examples/Fetch/controller/fetch_controller.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace drake {

using systems::Context;
using systems::ContinuousState;
using systems::RigidBodyPlant;
using systems::VectorBase;

namespace examples {
namespace Fetch {
namespace {

DEFINE_double(simulation_sec, 2, "Number of seconds to simulate");
DEFINE_double(contact_stiff, 20000, "Contact Stiffness");
DEFINE_double(contact_diss, 2, "Contact dissipation");

int DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_sec > 0);
  DRAKE_DEMAND(FLAGS_contact_stiff > 0);
  DRAKE_DEMAND(FLAGS_contact_diss > 0);

  drake::lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;

  // Adds a plant.
  RigidBodyPlant<double>* plant = nullptr;
  FetchControllerSystem<double>* controller = nullptr;

  const std::string kModelPath =
      drake::GetDrakePath() +
      "/examples/Fetch/fetch_description/robots/fetch.urdf";
  {
    auto tree = std::make_unique<RigidBodyTree<double>>();
    drake::multibody::AddFlatTerrainToWorld(tree.get());
    CreateTreeFromFloatingModelAtPose(kModelPath, tree.get(),
                                      Vector3<double>(0, 0, 0));

    std::cout << tree->get_num_actuators() << "\n";
    for (int i = 0; i < tree->get_num_actuators(); ++i) {
      std::cout << i << ": " << tree->actuators[i].name_ << "\n";
    }

    for (int i = 0; i < tree->get_num_positions(); ++i) {
      std::cout << i << ": " << tree->get_position_name(i) << "\n";
    }
    auto control_tree = tree->Clone();

    plant = builder.AddSystem<RigidBodyPlant<double>>(std::move(tree));
    plant->set_name("plant");
    plant->set_normal_contact_parameters(FLAGS_contact_stiff,
                                         FLAGS_contact_diss);

    controller = builder.AddSystem<FetchControllerSystem<double>>(std::move(control_tree));
  }

  // Verifies the tree.
  const RigidBodyTree<double>& tree = plant->get_rigid_body_tree();
  // VerifyFetchTree(tree);

  // Creates and adds LCM publisher for visualization.
  auto visualizer = builder.AddSystem<systems::DrakeVisualizer>(tree, &lcm);

  // Connects the visualizer and builds the diagram.
  builder.Connect(plant->get_output_port(0), visualizer->get_input_port(0));

  // Connects the controller and the plant.
  builder.Connect(controller->get_output_port_control(),
                  plant->actuator_command_input_port());
  builder.Connect(plant->get_output_port(0),
                  controller->get_input_port_full_estimated_state());

  // Visualize contacts.
  systems::ContactResultsToLcmSystem<double>& contact_viz =
    *builder.template AddSystem<systems::ContactResultsToLcmSystem<double>>(
        tree);
  contact_viz.set_name("contact_viz");

  auto& contact_results_publisher = *builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", &lcm));
  contact_results_publisher.set_name("contact_results_publisher");

  builder.Connect(plant->contact_results_output_port(),
      contact_viz.get_input_port(0));
  builder.Connect(contact_viz.get_output_port(0),
      contact_results_publisher.get_input_port(0));

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);

  Context<double>* fetch_context = diagram->GetMutableSubsystemContext(
      simulator.get_mutable_context(), plant);

  // Sets torso lift initial conditions.
  // See the @file docblock in fetch_common.h for joint index descriptions.
  VectorBase<double>* x0 = fetch_context->get_mutable_continuous_state_vector();
  x0->SetAtIndex(2, 0.2);  // base z [m]

  simulator.Initialize();

  // Simulate for the desired duration.
  simulator.set_target_realtime_rate(1);
  simulator.StepTo(10000);

  // Ensures the simulation was successful.
  const Context<double>& context = simulator.get_context();
  const ContinuousState<double>* state = context.get_continuous_state();
  const VectorBase<double>& position_vector = state->get_generalized_position();
  const VectorBase<double>& velocity_vector = state->get_generalized_velocity();

  const int num_q = position_vector.size();
  const int num_v = velocity_vector.size();

  // Ensures the sizes of the position and velocity vectors are correct.
  DRAKE_DEMAND(num_q == plant->get_num_positions());
  DRAKE_DEMAND(num_v == plant->get_num_velocities());
  DRAKE_DEMAND(num_q == num_v + 1);

  return 0;
}

}  // namespace
}  // namespace Fetch
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::Fetch::DoMain();
}
