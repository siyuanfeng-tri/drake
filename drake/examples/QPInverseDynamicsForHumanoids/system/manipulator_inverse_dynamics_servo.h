#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/manipulator_plan_eval_system.h"
#include "drake/systems/controllers/model_based_controller_base.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * Extends systems::ModelBasedController to an inverse dynamics controller for
 * robot manipulators that tracks given desired position, velocity and
 * acceleration.
 */
class ManipulatorInverseDynamicsServo : public systems::ModelBasedController<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ManipulatorInverseDynamicsServo)

  /**
   * Constructs a inverse dynamics controller for a manipulator arm.
   * @param model_path Path to the arm model, from which the internal
   * RigidBodyTree model is instantiated.
   * @param alias_group_path Path to the alias groups file. Used by the
   * controller to understand to topology of the robot.
   * @param controller_config_path Path the config file for the controller.
   * @param world_offset RigidBodyFrame X_WB, where B is the base of the robot.
   */
  ManipulatorInverseDynamicsServo(
      const std::string& model_path, const std::string& alias_group_path,
      const std::string& controller_config_path,
      std::shared_ptr<RigidBodyFrame<double>> world_offset = nullptr);

  /**
   * Initializes the controller's internal states.
   */
  void Initialize(systems::Context<double>* context);

  /**
   * Returns the input port for the desired acceleration.
   */
  const systems::InputPortDescriptor<double>& get_input_port_desired_acceleration() const {
    return systems::Diagram<double>::get_input_port(2);
  }

  /**
   * Returns the output port for debugging information from plan eval.
   */
  const systems::OutputPortDescriptor<double>& get_output_port_plan_eval_debug_info() const {
    return systems::Diagram<double>::get_output_port(1);
  }

  /**
   * Returns the output port for debugging information from inverse dynamics.
   */
  const systems::OutputPortDescriptor<double>& get_output_port_inverse_dynamics_debug_info() const {
    return systems::Diagram<double>::get_output_port(2);
  }

 private:
  ManipulatorPlanEvalSystem* servo_{nullptr};
  const double kControlDt{0.005};
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
