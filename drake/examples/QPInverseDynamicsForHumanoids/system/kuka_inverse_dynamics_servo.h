#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_path.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/joint_level_controller_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/kuka_servo_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/qp_controller_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/robot_status_wrapper.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/selector.h"

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
      std::unique_ptr<RigidBodyTree<double>> tree,
      int model_instance_idx,
      const std::string& alias_group_path,
      const std::string& controller_config_path) {
    Assemble(std::move(tree), model_instance_idx, alias_group_path, controller_config_path);
  }

  KukaInverseDynamicsServo(
      const std::string& model_path, const std::string& alias_group_path,
      const std::string& controller_config_path) {
    auto tree = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        model_path, drake::multibody::joints::kFixed, nullptr, tree.get());

    Assemble(std::move(tree), RigidBodyTreeConstants::kFirstNonWorldModelInstanceId, alias_group_path, controller_config_path);
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

  const RigidBodyTree<double>& get_robot_for_control() const { return *robot_for_control_; }

 private:
  void Assemble(std::unique_ptr<RigidBodyTree<double>> tree,
      int model_instance_idx,
      const std::string& alias_group_path,
      const std::string& controller_config_path) {
    this->set_name("KukaServoSim");

    drake::multibody::AddFlatTerrainToWorld(tree.get());

    systems::DiagramBuilder<double> builder;

    plant_ = builder.AddSystem<systems::RigidBodyPlant<double>>(move(tree));

    // TODO(siyuan): THIS IS A HACK
    robot_for_control_ = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
        multibody::joints::kFixed, robot_for_control_.get());
    const RigidBodyTree<double>& robot = *robot_for_control_;

    rs_wrapper_ = builder.AddSystem(
        std::make_unique<RobotStatusWrapper>(robot, alias_group_path));
    joint_level_controller_ =
        builder.AddSystem(std::make_unique<JointLevelControllerSystem>(robot));

    servo_ = builder.AddSystem(std::make_unique<KukaServoSystem>(
        robot, alias_group_path, controller_config_path, 0.002));
    id_controller_ =
        builder.AddSystem(std::make_unique<QPControllerSystem>(robot, 0.002));

    // Split out only the states that are relevant to the controller.
    std::vector<int> robot_state_indices;
    robot_state_indices.reserve(robot.get_num_positions() + robot.get_num_velocities());
    int robot_pos_start = plant_->get_model_position_index_start(model_instance_idx);
    int robot_vel_start = plant_->get_model_velocity_index_start(model_instance_idx);
    for (int i = 0; i < robot.get_num_positions(); i++)
      robot_state_indices.push_back(i + robot_pos_start);
    for (int i = 0; i < robot.get_num_velocities(); i++)
      robot_state_indices.push_back(i + robot_vel_start + plant_->get_num_positions());

    auto rbp_state_select = builder.AddSystem<systems::Selector<double>>(
        plant_->state_output_port().size(), robot_state_indices);

    // plant -> rs
    builder.Connect(plant_->state_output_port(), rbp_state_select->get_input_port(0));
    builder.Connect(rbp_state_select->get_output_port(0),
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
                    plant_->model_instance_actuator_command_input_port(model_instance_idx));

    // Exports plant's state output.
    builder.ExportOutput(plant_->state_output_port());
    builder.ExportOutput(rbp_state_select->get_output_port(0));

    builder.ExportInput(servo_->get_input_port_desired_state());

    // Exports torque command.
    builder.ExportOutput(joint_level_controller_->get_output_port_torque());

    builder.BuildInto(this);
  }

  std::unique_ptr<RigidBodyTree<double>> robot_for_control_{nullptr};

  systems::RigidBodyPlant<double>* plant_{nullptr};
  RobotStatusWrapper* rs_wrapper_{nullptr};
  JointLevelControllerSystem* joint_level_controller_{nullptr};
  KukaServoSystem* servo_{nullptr};
  QPControllerSystem* id_controller_{nullptr};
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
