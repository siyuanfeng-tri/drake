#pragma once

#include <memory>
#include <string>
#include <utility>

#include "bot_core/atlas_command_t.hpp"
#include "bot_core/robot_state_t.hpp"

#include "drake/examples/QPInverseDynamicsForHumanoids/system/joint_level_controller_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/new_manipulator_plan_eval_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/humanoid_status_translator_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/qp_controller_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/controllers/model_based_controller_base.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * A controller for humanoid balancing built on top of ManipulatorPlanEvalSystem
 * and QpControllerSystem. This diagram does not have any input or output ports.
 * The state inputs and control outputs are sent through LCM messages directly.
 */
class IiwaController
    : public systems::ModelBasedController<double> {
 public:
  IiwaController(
      const std::string& model_path, const std::string& alias_group_path,
      const std::string& controller_config_path, double dt,
      std::shared_ptr<RigidBodyFrame<double>> world_offset,
      const std::string& obj_model_path,
      std::shared_ptr<RigidBodyFrame<double>> obj_world_offset)
      : systems::ModelBasedController<double>(model_path, world_offset,
                                              multibody::joints::kFixed) {

    obj_ = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFile(
        obj_model_path, multibody::joints::kQuaternion, obj_world_offset, obj_.get());

    const RigidBodyTree<double>& robot = get_robot_for_control();
    systems::DiagramBuilder<double> builder;

    // Converts raw state to humanoid status.
    StateToHumanoidStatusSystem* rs_wrapper =
        builder.AddSystem<StateToHumanoidStatusSystem>(robot, alias_group_path);
    rs_wrapper->set_name("rs_wrapper");

    // Converts qp output to raw torque.
    TrivialJointLevelControllerSystem* joint_level_controller =
        builder.AddSystem<TrivialJointLevelControllerSystem>(robot);
    joint_level_controller->set_name("joint_level_controller");

    // PlanEval + controller
    plan_eval_ = builder.AddSystem(std::make_unique<ManipulatorPlanEvalSystem>(
        robot, *obj_, alias_group_path, controller_config_path, dt));
    plan_eval_->set_name("plan_eval");

    QpControllerSystem* qp_con = builder.AddSystem(
        std::make_unique<QpControllerSystem>(robot, dt));
    qp_con->set_name("qp_con");

    // IO
    int index = builder.ExportInput(rs_wrapper->get_input_port_state());
    this->set_input_port_index_estimated_state(index);

    input_index_plan_ = builder.ExportInput(plan_eval_->get_input_port_plan());
    input_index_obj_state_ = builder.ExportInput(
        plan_eval_->get_input_port_object_state());

    index =
      builder.ExportOutput(joint_level_controller->get_output_port_torque());
    this->set_output_port_index_control(index);

    output_index_plan_eval_debug_ = builder.ExportOutput(plan_eval_->get_output_port_debug_info());

    output_index_qp_debug_ = builder.ExportOutput(qp_con->get_output_port_debug_info());

    // rs -> qp_input
    builder.Connect(rs_wrapper->get_output_port_humanoid_status(),
                    plan_eval_->get_input_port_humanoid_status());
    // rs + qp_input -> qp_output
    builder.Connect(rs_wrapper->get_output_port_humanoid_status(),
                    qp_con->get_input_port_humanoid_status());
    builder.Connect(plan_eval_->get_output_port_qp_input(),
                    qp_con->get_input_port_qp_input());
    // qp_output -> atlas_command_t
    builder.Connect(qp_con->get_output_port_qp_output(),
                    joint_level_controller->get_input_port_qp_output());

    builder.BuildInto(this);
  }

  /**
   * Returns a pointer to the plan eval block.
   */
  ManipulatorPlanEvalSystem* get_mutable_plan_eval() { return plan_eval_; }

  /**
   * Initializes the controller's internal state. Must be called before
   * execution.
   */
  void Initialize(double time,
      const VectorX<double>& q, const VectorX<double>& v,
      systems::Context<double>* context) {
    HumanoidStatus status(get_robot_for_control(), get_alias_groups());
    status.UpdateKinematics(time, q, v);
    plan_eval_->Initialize(status, context->get_mutable_state());
  }

  /**
   * Returns plan eval's alias groups.
   */
  const param_parsers::RigidBodyTreeAliasGroups<double>& get_alias_groups()
      const {
    return plan_eval_->get_alias_groups();
  }

  /**
   * Returns plan eval's parameters.
   */
  const param_parsers::ParamSet& get_paramset() const {
    return plan_eval_->get_paramset();
  }

  const systems::InputPortDescriptor<double>&
  get_input_port_plan() const {
    return systems::Diagram<double>::get_input_port(
        input_index_plan_);
  }

  const systems::InputPortDescriptor<double>&
  get_input_port_object_state() const {
    return systems::Diagram<double>::get_input_port(
        input_index_obj_state_);
  }

  const systems::OutputPortDescriptor<double>&
  get_output_port_plan_eval_debug_info() const {
    return get_output_port(output_index_plan_eval_debug_);
  }

  const systems::OutputPortDescriptor<double>&
  get_output_port_qp_debug_info() const {
    return get_output_port(output_index_qp_debug_);
  }

 private:
  ManipulatorPlanEvalSystem* plan_eval_{nullptr};

  std::unique_ptr<RigidBodyTree<double>> obj_;

  int input_index_plan_;
  int input_index_obj_state_;
  int output_index_plan_eval_debug_;
  int output_index_qp_debug_;
};

}  // end namespace qp_inverse_dynamics
}  // end namespace examples
}  // end namespace drake
