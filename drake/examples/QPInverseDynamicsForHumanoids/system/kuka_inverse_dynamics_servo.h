#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/examples/QPInverseDynamicsForHumanoids/system/joint_level_controller_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/kuka_servo_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/qp_controller_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/robot_status_wrapper.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * Builds a Diagram of a Kuka IIWA arm controlled by inverse dynamics to follow
 * a desired trajectory.
 */
class KukaInverseDynamicsServo : public systems::Diagram<double> {
 public:
  KukaInverseDynamicsServo(
      const std::string& model_path, const std::string& alias_group_path,
      const std::string& controller_config_path) {
    this->set_name("KukaServoSim");

    auto tree = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        model_path, drake::multibody::joints::kFixed, nullptr, tree.get());

    drake::multibody::AddFlatTerrainToWorld(tree.get());

    systems::DiagramBuilder<double> builder;

    plant_ = builder.AddSystem<systems::RigidBodyPlant<double>>(move(tree));
    const RigidBodyTree<double>& robot = plant_->get_rigid_body_tree();

    rs_wrapper_ = builder.AddSystem(
        std::make_unique<RobotStatusWrapper>(robot, alias_group_path));
    joint_level_controller_ =
        builder.AddSystem(std::make_unique<JointLevelControllerSystem>(robot));

    servo_ = builder.AddSystem(std::make_unique<KukaServoSystem>(
        robot, alias_group_path, controller_config_path, 0.002));
    id_controller_ =
        builder.AddSystem(std::make_unique<QPControllerSystem>(robot, 0.002));

    viz_publisher_ =
        builder.template AddSystem<systems::DrakeVisualizer>(robot, &lcm_);

    // plant -> rs
    builder.Connect(plant_->state_output_port(),
                    rs_wrapper_->get_input_port_state());

    // rs -> qp_input
    builder.Connect(rs_wrapper_->get_output_port_humanoid_status(),
                    servo_->get_input_port_humanoid_status());
    // rs + qp_input -> qp_output
    builder.Connect(rs_wrapper_->get_output_port_humanoid_status(),
                    id_controller_->get_input_port_humanoid_status());
    builder.Connect(servo_->get_output_port_qp_input(),
                    id_controller_->get_input_port_qp_input());
    // qp_output -> joint controller
    builder.Connect(id_controller_->get_output_port_qp_output(),
                    joint_level_controller_->get_input_port_qp_output());
    // joint controller -> plant
    builder.Connect(joint_level_controller_->get_output_port_torque(),
                    plant_->get_input_port(0));

    // plant -> viz
    builder.Connect(plant_->state_output_port(),
                    viz_publisher_->get_input_port(0));

    // Exports plant's state output.
    builder.ExportOutput(plant_->state_output_port());

    builder.ExportInput(servo_->get_input_port_desired_state());

    // Exports torque command.
    builder.ExportOutput(joint_level_controller_->get_output_port_torque());

    builder.BuildInto(this);
  }

  const systems::RigidBodyPlant<double>& get_plant() const {
    return *plant_;
  }

  void Initialize(systems::Context<double>* context) {
    systems::Context<double>* servo_context =
        GetMutableSubsystemContext(context, servo_);
    systems::State<double>* servo_state =
        servo_context->get_mutable_state();
    servo_->Initialize(servo_state);
  }

  systems::Context<double>* get_kuka_context(
      systems::Context<double>* context) const {
    return GetMutableSubsystemContext(context, plant_);
  }

 private:
  systems::RigidBodyPlant<double>* plant_{nullptr};
  RobotStatusWrapper* rs_wrapper_{nullptr};
  JointLevelControllerSystem* joint_level_controller_{nullptr};
  KukaServoSystem* servo_{nullptr};
  QPControllerSystem* id_controller_{nullptr};

  std::unique_ptr<PiecewisePolynomialTrajectory> poly_trajectory_;
  systems::DrakeVisualizer* viz_publisher_{nullptr};
  drake::lcm::DrakeLcm lcm_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
