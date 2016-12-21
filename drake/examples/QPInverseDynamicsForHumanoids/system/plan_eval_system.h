#pragma once

#include <string>

#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/example_qp_input_for_valkyrie.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/manip_plan.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/walking_plan.h"
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
  explicit PlanEvalSystem(const RigidBodyTree<double>& robot)
    : robot_(robot), manip_plan_eval_(HumanoidWalkingPlan(robot)), qp_input_(QPInput(robot)) {
    //: robot_(robot), manip_plan_eval_(HumanoidManipPlan(robot)), qp_input_(QPInput(robot)) {
    input_port_index_humanoid_status_ = DeclareAbstractInputPort().get_index();
    output_port_index_qp_input_ = DeclareAbstractOutputPort().get_index();

    set_name("plan_eval");
  }

  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {
    // Input:
    const HumanoidStatus* robot_status = EvalInputValue<HumanoidStatus>(
        context, input_port_index_humanoid_status_);

    // Output:
    lcmt_qp_input& msg = output->GetMutableData(output_port_index_qp_input_)
                             ->GetMutableValue<lcmt_qp_input>();

    // manip_plan_eval_.UpdateQPInput(*robot_status, &qp_input_);
    manip_plan_eval_.Control(*robot_status, &qp_input_);
    EncodeQPInput(qp_input_, &msg);
  }

  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const Context<double>& context) const override {
    std::unique_ptr<LeafSystemOutput<double>> output(
        new LeafSystemOutput<double>);
    output->add_port(std::unique_ptr<AbstractValue>(
        new Value<lcmt_qp_input>(lcmt_qp_input())));
    return std::move(output);
  }

  /**
   * Set the set point for tracking.
   * @param robot_status, desired robot state
   */
  void SetDesired(const HumanoidStatus& robot_status) {
    manip_plan_eval_.HandleWalkingPlan(robot_status, &qp_input_);
    //manip_plan_eval_.HandleManipPlan(robot_status, &qp_input_);
  }

  /**
   * @return Port for the input: HumanoidStatus.
   */
  inline const InputPortDescriptor<double>& get_input_port_humanoid_status()
      const {
    return get_input_port(input_port_index_humanoid_status_);
  }

  /**
   * @return Port for the output: QPInput.
   */
  inline const OutputPortDescriptor<double>& get_output_port_qp_input() const {
    return get_output_port(output_port_index_qp_input_);
  }

 private:
  const RigidBodyTree<double>& robot_;

  int input_port_index_humanoid_status_;
  int output_port_index_qp_input_;

  // Gains and setpoints.
  //HumanoidManipPlan manip_plan_eval_;
  mutable HumanoidWalkingPlan manip_plan_eval_;
  mutable QPInput qp_input_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
