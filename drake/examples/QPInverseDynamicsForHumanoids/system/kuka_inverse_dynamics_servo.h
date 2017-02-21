#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/examples/QPInverseDynamicsForHumanoids/system/kuka_servo_system.h"
#include "drake/systems/controllers/model_based_controller_base.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * Builds a Diagram of a Kuka IIWA arm controlled by inverse dynamics to follow
 * a desired trajectory.
 */
class KukaInverseDynamicsServo : public systems::ModelBasedController<double> {
 public:
  /**
   * Constructs a inverse dynamics controller for the Kuka iiwa arm. It
   * maintains a separate RigidBodyTree just for the controller, which can be
   * instantiated with different model file than the one used for simulation.
   * @param model_path Path to the Kuka iiwa model, from which the internal
   * model is instantiated.
   * @param alias_group_path Path to the alias groups file. Used by the
   * controller to understand to topology of the robot.
   * @param controller_config_path Path the config file for the controller.
   * @param world_offset RigidBodyFrame X_WB, where B is the base of the robot.
   */
  KukaInverseDynamicsServo(
      const std::string& model_path, const std::string& alias_group_path,
      const std::string& controller_config_path,
      std::shared_ptr<RigidBodyFrame<double>> world_offset = nullptr);

  void Initialize(systems::Context<double>* context);

  const systems::InputPortDescriptor<double>& get_input_port_desired_acceleration() const {
    return systems::Diagram<double>::get_input_port(2);
  }

  const systems::OutputPortDescriptor<double>& get_output_port_debug_info() const {
    return systems::Diagram<double>::get_output_port(1);
  }

 private:
  KukaServoSystem* servo_{nullptr};
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
