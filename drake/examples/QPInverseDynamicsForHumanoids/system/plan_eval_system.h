#pragma once

#include <memory>

#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/param_parsers/param_parser.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

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
class PlanEvalSystem : public systems::LeafSystem<double> {
 public:
  explicit PlanEvalSystem(const RigidBodyTree<double>& robot);

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  std::unique_ptr<systems::SystemOutput<double>> AllocateOutput(
      const systems::Context<double>& context) const override;

  /**
   * Set the set point for tracking.
   * @param robot_status, desired robot state
   */
  void SetDesired(const HumanoidStatus& robot_status);

  /**
   * @return Port for the input: HumanoidStatus.
   */
  inline const systems::InputPortDescriptor<double>&
  get_input_port_humanoid_status() const {
    return get_input_port(input_port_index_humanoid_status_);
  }

  /**
   * @return Port for the output: QPInput.
   */
  inline const systems::OutputPortDescriptor<double>& get_output_port_qp_input()
      const {
    return get_output_port(output_port_index_qp_input_);
  }

 private:
  const RigidBodyTree<double>& robot_;
  param_parsers::RigidBodyTreeAliasGroups<double> alias_groups_;
  param_parsers::ParamSet paramset_;

  int input_port_index_humanoid_status_;
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
  VectorX<double> Kp_dof_;
  VectorX<double> Kd_dof_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
