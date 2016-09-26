#pragma once

#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/translator_between_lcmt_drake_signal.h"
#include "drake/lcmt_drake_signal.hpp"

#include "../qp_controller.h"
#include "qp_io_utils.h"

namespace drake {
namespace systems {

class System2QP : public LeafSystem<double> {
 public:
  explicit System2QP(const RigidBodyTree &robot) :
    robot_(robot) {
    // make IO ports
    // state input
    this->DeclareAbstractInputPort(kInheritedSampling);

    // qp input
    this->DeclareAbstractInputPort(kInheritedSampling);

    // qp output
    this->DeclareAbstractOutputPort(kInheritedSampling);

    set_name("qp_controller");
  }

  void EvalOutput(const Context<double> &context, SystemOutput<double>* output) const override {
    DRAKE_ASSERT(context.get_num_input_ports() == 2);
    DRAKE_ASSERT(output->get_num_ports() == 1);

    // get robot status
    const HumanoidStatus &rs = context.get_abstract_input(0)->GetValue<HumanoidStatus>();

    DRAKE_ASSERT(rs.robot().number_of_positions() == robot_.number_of_positions());
    DRAKE_ASSERT(rs.robot().number_of_velocities() == robot_.number_of_velocities());
    DRAKE_ASSERT(rs.robot().actuators.size() == robot_.actuators.size());

    // get qp input
    const AbstractValue* abs_in = context.get_abstract_input(1);
    const QPInput &qp_input = abs_in->GetValue<QPInput>();

    // qp output
    QPOutput qp_output(rs.robot());

    QPController qp_controller(rs, 4);

    if (qp_controller.Control(rs, qp_input, &qp_output) < 0) {
      throw std::runtime_error("System2QP: QP canot solve\n");
    }

    output->GetMutableData(0)->SetValue(qp_output);
  }

  std::unique_ptr<SystemOutput<double>> AllocateOutput(const Context<double>& context) const {
    std::unique_ptr<LeafSystemOutput<double>> output(new LeafSystemOutput<double>);
    QPOutput out(robot_);
    output->add_port(std::unique_ptr<AbstractValue>(new Value<QPOutput>(out)));
    return std::unique_ptr<SystemOutput<double>>(output.release());
  }

 private:
  const RigidBodyTree &robot_;
};


}
}
