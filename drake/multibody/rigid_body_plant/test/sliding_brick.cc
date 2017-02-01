#include <iomanip>
#include <limits>
#include <memory>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/explicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include <drake/systems/framework/diagram_builder.h>
#include <drake/multibody/rigid_body_plant/drake_visualizer.h>
#include <drake/systems/primitives/constant_vector_source.h>

namespace drake {
namespace systems {

using drake::lcm::DrakeLcm;
using drake::multibody::joints::kFixed;
using Eigen::VectorXd;
using std::make_unique;

// Simple scenario of one block sliding down another block.
int main(int argc, const char**argv) {

  // Process arguments
  double xdot_0 = 0;
  double timestep = 0.001;
  double push = 1.0;
  double stiffness = 100000;
  double static_coeff = 1.5;
  double dynamic_coeff = 1.0;
  double slip_threshold = 0.01;
  double dissipation = 1.0;
  if ( argc == 4 ) {
    timestep = std::atof(argv[1]);
    push = std::atof(argv[2]);
    xdot_0 = std::atof(argv[3]);
  } else {
    std::cout << "Expected three command-line arguments: time step, push "
    "magnitude, and initial speed in push direction. In the absence of those "
    "values, using defaults.\n";
  }
  std::cout << "Paramters:\n";
  std::cout << "\tTimetep:          " << timestep << "\n";
  std::cout << "\tPush:             " << push << "\n";
  std::cout << "\tẋ:                " << xdot_0 << "\n";
  std::cout << "\tStiffness:        " << stiffness << "\n";
  std::cout << "\tStatic friction:  " << static_coeff << "\n";
  std::cout << "\tDynamic friction: " << dynamic_coeff << "\n";
  std::cout << "\tSlip Threshold:   " << slip_threshold << "\n";
  std::cout << "\tDissipation:      " << dissipation << "\n";

  DiagramBuilder<double> builder;

  // Create RigidBodyTree.
  auto tree_ptr = make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      drake::GetDrakePath() +
          "/multibody/rigid_body_plant/test/SlidingBrick.urdf",
      kFixed, nullptr /* weld to frame */, tree_ptr.get());
  multibody::AddFlatTerrainToWorld(tree_ptr.get(), 100., 10.);

  // Instantiate a RigidBodyPlant from the RigidBodyTree.
  auto& plant = *builder.AddSystem<RigidBodyPlant<double>>(move(tree_ptr));
  // Contact parameters set arbitrarily.
#ifdef USE_STRIBECK
  plant.set_contact_parameters(stiffness, static_coeff, dynamic_coeff,
                               slip_threshold, dissipation);
#else
  plant.set_contact_parameters(10000., 100., 10.);
#endif
  const auto& tree = plant.get_rigid_body_tree();

  // RigidBodyActuators.
  DRAKE_ASSERT(tree.actuators.size() == 1u);

  // LCM communication.
  DrakeLcm lcm;

  // Pusher
  VectorXd push_value(1); // the value for the prismatic joint.
  push_value << push;
  const ConstantVectorSource<double>& push_source =
      *builder.template AddSystem<ConstantVectorSource<double>>( push_value );

  builder.Connect(push_source.get_output_port(), plant.get_input_port(0));

  // Visualizer.
  const DrakeVisualizer& visualizer_publisher =
      *builder.template AddSystem<DrakeVisualizer>(tree, &lcm);

  // Raw state vector to visualizer.
  builder.Connect(plant.state_output_port(),
                  visualizer_publisher.get_input_port(0));

  auto diagram = builder.Build();


  // Create simulator.
  auto simulator = std::make_unique<Simulator<double>>(*diagram);
  auto context = simulator->get_mutable_context();
  // Integrator set arbitrarily.
//  simulator->reset_integrator<ExplicitEulerIntegrator<double>>(*diagram, timestep,
//                                                               context);
  simulator->reset_integrator<RungeKutta2Integrator<double>>(*diagram, timestep, context);

//  simulator->reset_integrator<RungeKutta3Integrator<double>>(*diagram, context);
//  simulator->get_mutable_integrator()->set_maximum_step_size(1e-3);
//  simulator->get_mutable_integrator()->set_target_accuracy(1e-2);

  // Set initial state.
  auto plant_context = diagram->GetMutableSubsystemContext(context, &plant);

  // TODO(tkoolen): make it easy to specify a different initial configuration.

  // 6 1-dof joints = 6 * (x, ẋ)
  const int kStateSize = 12;
  VectorX<double> initial_state(kStateSize);
  initial_state << 0, 0, 0, 0, 0, 0, xdot_0, 0, 0, 0, 0, 0;
      plant.set_state_vector(plant_context, initial_state);

  // Acquire access to bodies for output
  RigidBody<double>* brick = plant.get_rigid_body_tree().FindBody("brick", "", 0);
  RigidBody<double>* xtrans = plant.get_rigid_body_tree().FindBody("xtrans", "", 0);

  auto output = plant.AllocateOutput(*plant_context);
  const int port_index = plant.kinematics_results_output_port().get_index();

  std::cout << "Time,  " << brick->get_name() << " x, x velocity, |f_N|, |f_T|\n";
  double time = 0.0;
  while (time < 5.0) {
    time = context->get_time();
    SPDLOG_TRACE(drake::log(), "Time is now {}", time);
    plant.CalcOutput(*plant_context, output.get());
    auto results = output->get_data(port_index)->GetValue<KinematicsResults<double>>();
    std::cout << std::setprecision(3) << time;
    std::cout << " " << std::setprecision(15) << results.get_pose_in_world(*brick).translation()(0);
    std::cout << " " << results.get_joint_velocity(*xtrans)(0);

    auto contacts = output->get_data(plant.contact_results_output_port().get_index())->GetValue<ContactResults<double>>();
    if ( contacts.get_num_contacts() > 0 ) {
      // This assumes that all contacts are between the collision points on the
      // box and the ground plane.  That they all share the same normal and
      // can simply be summed up.
      Vector3<double> net_force = Vector3<double>::Zero();
      for (int i = 0; i < contacts.get_num_contacts(); ++i) {
        const auto& info = contacts.get_contact_info(i);
        const auto& force = info.get_resultant_force();
        net_force += force.get_force();
      }
      std::cout << " " << std::abs(net_force(2));
      std::cout << " " << net_force.template head<2>().norm();
    } else {
      std::cout << " 0 0";
    }

    std::cout << "\n";
    std::cout.flush();
    simulator->StepTo(time + 0.01);
  }

  return 0;
}
}  // namespace systems
}  // namespace drake

int main(int argc, const char* argv[]) {
  return drake::systems::main(argc, argv);
}
