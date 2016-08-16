#pragma once

#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/translator_between_lcmt_drake_signal.h"
#include "drake/lcmt_drake_signal.hpp"

namespace drake {
namespace systems {

class System2QP : public LeafSystem<double> {
 public:
  System2QP() {
    // state input
    DeclareInputPort(kVectorValued, num_states, kInheritedSampling);
    // qp input
    DeclareInputPort(kVectorValued, num_qp_input, kInheritedSampling);

    // qdd out
    DeclareOutputPort(kVectorValued, num_qdd, kInheritedSampling);
    // trq out
    DeclareOutputPort(kVectorValued, num_trq, kInheritedSampling);
  }

  void EvalOutput(const ContextBase<double> &context, SystemOutput<double>* output) const override;

  const static int num_states = 36;
  const static int num_qp_input = 30;
  const static int num_trq = 30;
  const static int num_qdd = 36;
};

void testQP(::lcm::LCM &lcm);


}
}
