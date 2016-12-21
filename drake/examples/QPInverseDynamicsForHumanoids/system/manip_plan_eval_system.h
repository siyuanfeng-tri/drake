#pragma once

#include <string>

#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/example_qp_input_for_valkyrie.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"
#include "drake/systems/framework/leaf_system.h"

#include "robotlocomotion/robot_plan_t.hpp"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

using systems::Context;
using systems::SystemOutput;
using systems::SystemPortDescriptor;
using systems::LeafSystemOutput;
using systems::AbstractValue;
using systems::Value;

// TODO(siyuan.feng): Extend this class properly to support various different
// plans. This class currently only supports tracking a stationary fixed point.

/**
 * A simple PlanEval block that generates qp input for the qp inverse dynamics
 * controller.
 * The controller is set up to track a stationary fixed point assuming the
 * robot is in double support, and the desired set point is set by SetDesired.
 *
 * Input: HumanoidStatus
 * Output: QPInput
 */
class ManipPlanEvalSystem : public systems::LeafSystem<double> {
 public:
  ManipPlanEvalSystem(const RigidBodyTree<double>& robot);

  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override;

  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const Context<double>& context) const override {
    std::unique_ptr<LeafSystemOutput<double>> output(
        new LeafSystemOutput<double>);
    output->add_port(std::unique_ptr<AbstractValue>(
        new Value<lcmt_qp_input>(lcmt_qp_input())));
    return std::move(output);
  }

  void HandleNewManipPlan(const robotlocomotion::robot_plan_t& plan) const;

  /**
   * Set the set point for tracking.
   * @param robot_status, desired robot state
   */
  void SetDesired(const HumanoidStatus& robot_status) {
    desired_com_ = robot_status.com();
    pelvis_PDff_ = CartesianSetpoint<double>(
        robot_status.pelvis().pose(), Vector6<double>::Zero(),
        Vector6<double>::Zero(), Kp_pelvis_, Kd_pelvis_);
    torso_PDff_ = CartesianSetpoint<double>(
        robot_status.torso().pose(), Vector6<double>::Zero(),
        Vector6<double>::Zero(), Kp_torso_, Kd_torso_);
    int dim = robot_status.position().size();
    joint_PDff_ = VectorSetpoint<double>(
        robot_status.position(), VectorX<double>::Zero(dim),
        VectorX<double>::Zero(dim), Kp_joints_, Kd_joints_);
  }

  /**
   * @return Port for the input: HumanoidStatus.
   */
  inline const SystemPortDescriptor<double>& get_input_port_manip_plan()
      const {
    return get_input_port(input_port_index_manip_plan_);
  }

  /**
   * @return Port for the input: HumanoidStatus.
   */
  inline const SystemPortDescriptor<double>& get_input_port_humanoid_status()
      const {
    return get_input_port(input_port_index_humanoid_status_);
  }

  /**
   * @return Port for the output: QPInput.
   */
  inline const SystemPortDescriptor<double>& get_output_port_qp_input() const {
    return get_output_port(output_port_index_qp_input_);
  }

 private:
  const RigidBodyTree<double>& robot_;

  int input_port_index_humanoid_status_;
  int input_port_index_manip_plan_;
  int output_port_index_qp_input_;

  // Gains and setpoints.
  VectorSetpoint<double> joint_PDff_;
  CartesianSetpoint<double> pelvis_PDff_;
  CartesianSetpoint<double> torso_PDff_;

  Vector3<double> desired_com_;
  Vector3<double> Kp_com_;
  Vector3<double> Kd_com_;

  Vector6<double> Kp_pelvis_;
  Vector6<double> Kd_pelvis_;
  Vector6<double> Kp_torso_;
  Vector6<double> Kd_torso_;
  VectorX<double> Kp_joints_;
  VectorX<double> Kd_joints_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
