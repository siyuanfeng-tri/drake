#include "drake/examples/QPInverseDynamicsForHumanoids/system/manipulator_inverse_dynamics_servo.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/examples/QPInverseDynamicsForHumanoids/system/joint_level_controller_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/qp_controller_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/state_to_humanoid_status.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

ManipulatorInverseDynamicsServo::ManipulatorInverseDynamicsServo(
    const std::string& model_path, const std::string& alias_group_path,
    const std::string& controller_config_path,
    std::shared_ptr<RigidBodyFrame<double>> world_offset)
    : systems::ModelBasedController<double>(model_path, world_offset, multibody::joints::kFixed) {

  const RigidBodyTree<double>& robot = get_robot_for_control();

  this->set_name("ManipulatorInverseDynamicsServo");

  systems::DiagramBuilder<double> builder;

  // Converts raw state to humanoid status.
  StateToHumanoidStatus* rs_wrapper = builder.AddSystem(
      std::make_unique<StateToHumanoidStatus>(robot, alias_group_path));
  // Converts qp output to raw torque.
  JointLevelControllerSystem* joint_level_controller =
      builder.AddSystem(std::make_unique<JointLevelControllerSystem>(robot));
  // Generates qp input from desired q and qd qdd.
  servo_ = builder.AddSystem(std::make_unique<ManipulatorPlanEvalSystem>(
      robot, alias_group_path, controller_config_path, kControlDt));
  // Computes qp output from qp input.
  QPControllerSystem* id_controller =
    builder.AddSystem(std::make_unique<QPControllerSystem>(robot, kControlDt));

  // Connects estimate state to plan eval.
  builder.Connect(rs_wrapper->get_output_port_humanoid_status(),
      servo_->get_input_port_humanoid_status());

  // Connects estimated state to inverse dynamics.
  builder.Connect(rs_wrapper->get_output_port_humanoid_status(),
      id_controller->get_input_port_humanoid_status());

  // Connects plan eval to inverse dynamics.
  builder.Connect(servo_->get_output_port_qp_input(),
      id_controller->get_input_port_qp_input());

  // Connects inverse dynamics to joint controller.
  builder.Connect(id_controller->get_output_port_qp_output(),
      joint_level_controller->get_input_port_qp_output());

  // Combine qd, qd input and qdd input.
  std::vector<int> input_sizes =
      {robot.get_num_positions() + robot.get_num_velocities(),
       robot.get_num_velocities()};
  systems::Multiplexer<double>* mux =
      builder.AddSystem(std::make_unique<systems::Multiplexer<double>>(input_sizes));

  // Connects desired q, qd, qdd to plan eval.
  builder.Connect(mux->get_output_port(0),
                  servo_->get_input_port_desired_state_and_acceleration());

  // Exposes input: estimated arm state.
  builder.ExportInput(rs_wrapper->get_input_port_state());

  // Exposes input: desired q qd.
  builder.ExportInput(mux->get_input_port(0));

  // Exposes input: desired qdd.
  builder.ExportInput(mux->get_input_port(1));

  // Exposes output: computed torque.
  builder.ExportOutput(joint_level_controller->get_output_port_torque());

  // Exposes output: plan eval's debug output.
  builder.ExportOutput(servo_->get_output_port_debug_info());

  // Exposes output: inverse dynamics' debug output.
  builder.ExportOutput(id_controller->get_output_port_debug_info());

  builder.BuildInto(this);
}

void ManipulatorInverseDynamicsServo::Initialize(systems::Context<double>* context) {
  systems::Context<double>* servo_context =
    GetMutableSubsystemContext(context, servo_);
  systems::State<double>* servo_state = servo_context->get_mutable_state();
  servo_->Initialize(servo_state);
}


}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
