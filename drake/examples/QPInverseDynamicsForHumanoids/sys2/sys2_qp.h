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
  explicit System2QP() {
    // make IO ports
    // state input
    this->DeclareAbstractInputPort(kInheritedSampling);

    // qp input
    this->DeclareAbstractInputPort(kInheritedSampling);

    // qp output
    this->DeclareAbstractOutputPort(kInheritedSampling);
  }

  void EvalOutput(const ContextBase<double> &context, SystemOutput<double>* output) const override;

  std::unique_ptr<SystemOutput<double>> AllocateOutput(const ContextBase<double>& context) const {
    const HumanoidStatus &rs = context.get_abstract_input(0)->GetValue<HumanoidStatus>();

    std::unique_ptr<LeafSystemOutput<double>> output(new LeafSystemOutput<double>);
    QPOutput out(rs.robot());
    output->add_port(std::unique_ptr<AbstractValue>(new Value<QPOutput>(out)));
    return std::unique_ptr<SystemOutput<double>>(output.release());
  }

 private:
};


}
}
