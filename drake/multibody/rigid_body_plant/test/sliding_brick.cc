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

// Simulation parameters
double xdot_0 = 0;
double timestep = 0.001;
double push = 1.0;
double stiffness = 100000;
double static_coeff = 1.5;
double dynamic_coeff = 1.0;
double slip_threshold = 0.01;
double dissipation = 1.0;
double maxT = 2.5;
bool low_center_of_mass = false;  // if true, uses SlidingBrickLowCOm.urdf instead

void printUsage() {
  std::cout << "Pushes a sliding block across the ground.\n";
  std::cout << "\nParamters:\n";
  std::cout << "  -v [float]    Block initial velocity\n";
  std::cout << "  -dt [float]   Simulation time step\n";
  std::cout << "  -p [float]    Magnitude of pushing force\n";
  std::cout << "  -k [float]    Contact stiffness\n";
  std::cout << "  -us [float]   Static coefficient of friciton\n";
  std::cout << "  -ud [float]   Dynamic coefficient of friction\n";
  std::cout << "  -d [float]    Dissipation\n";
  std::cout << "  -vs [float]   Slip velocity threshold\n";
  std::cout << "  -T [float]    Maximum simulation duration\n";
  std::cout << "  -low          Use the brick with a *low* center of mass\n";
  std::cout << "  -h            Display this information\n";
}


void parseArgs(int argc, const char** argv) {
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "-v") == 0) {
      ++i;
      xdot_0 = std::atof(argv[i]);
    } else if (std::strcmp(argv[i], "-p") == 0) {
      ++i;
      push = std::atof(argv[i]);
    } else if (std::strcmp(argv[i], "-dt") == 0) {
      ++i;
      timestep = std::atof(argv[i]);
    } else if (std::strcmp(argv[i], "-k") == 0) {
      ++i;
      stiffness = std::atof(argv[i]);
    } else if (std::strcmp(argv[i], "-us") == 0) {
      ++i;
      static_coeff = std::atof(argv[i]);
    } else if (std::strcmp(argv[i], "-ud") == 0) {
      ++i;
      dynamic_coeff = std::atof(argv[i]);
    } else if (std::strcmp(argv[i], "-vs") == 0) {
      ++i;
      slip_threshold = std::atof(argv[i]);
    } else if (std::strcmp(argv[i], "-d") == 0) {
      ++i;
      dissipation = std::atof(argv[i]);
    } else if (std::strcmp(argv[i], "-T") == 0) {
      ++i;
      maxT = std::atof(argv[i]);
    } else if (std::strcmp(argv[i], "-low") == 0) {
      low_center_of_mass = true;
    } else if (std::strcmp(argv[i], "-h") == 0) {
      printUsage();
      exit(0);
    } else {
      std::cout << "Unrecognized command line parameter: " << argv[i] << ".\n";
      exit(1);
    }
  }
}

// Simple scenario of one block sliding down another block.
int main(int argc, const char**argv) {

  parseArgs(argc, argv);
  std::cout << "Paramters:\n";
  std::cout << "\tTimetep:          " << timestep << "\n";
  std::cout << "\tPush:             " << push << "\n";
  std::cout << "\tẋ:                " << xdot_0 << "\n";
  std::cout << "\tStiffness:        " << stiffness << "\n";
  std::cout << "\tStatic friction:  " << static_coeff << "\n";
  std::cout << "\tDynamic friction: " << dynamic_coeff << "\n";
  std::cout << "\tSlip Threshold:   " << slip_threshold << "\n";
  std::cout << "\tDissipation:      " << dissipation << "\n";
  std::cout << "\tUsing low CoM:    " << low_center_of_mass << "\n";

  DiagramBuilder<double> builder;

  // Create RigidBodyTree.
  auto tree_ptr = make_unique<RigidBodyTree<double>>();
  if (!low_center_of_mass) {
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        drake::GetDrakePath() +
            "/multibody/rigid_body_plant/test/SlidingBrick.urdf",
        kFixed, nullptr /* weld to frame */, tree_ptr.get());
  } else {
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        drake::GetDrakePath() +
            "/multibody/rigid_body_plant/test/SlidingBrickLowCom.urdf",
        kFixed, nullptr /* weld to frame */, tree_ptr.get());
  }
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
  RigidBody<double>* yrot = plant.get_rigid_body_tree().FindBody("yrot", "", 0);

  auto output = plant.AllocateOutput(*plant_context);
  const int port_index = plant.kinematics_results_output_port().get_index();

  std::cout << "Time,  " << brick->get_name() << " x, x velocity, y rot, |f_N|, |f_T|\n";
  double time = 0.0;
  while (time < maxT) {
    time = context->get_time();
    SPDLOG_TRACE(drake::log(), "Time is now {}", time);
    plant.CalcOutput(*plant_context, output.get());
    auto results = output->get_data(port_index)->GetValue<KinematicsResults<double>>();
    std::cout << std::setprecision(16) << time;
    std::cout << " " << std::setprecision(16) << results.get_pose_in_world(*brick).translation()(0);
    std::cout << " " << results.get_joint_velocity(*xtrans)(0);
    std::cout << " " << results.get_joint_position(*yrot)(0);

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
    simulator->StepTo(time + timestep);
  }

  return 0;
}
}  // namespace systems
}  // namespace drake

int main(int argc, const char* argv[]) {
  return drake::systems::main(argc, argv);
}
