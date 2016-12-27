#pragma once

#include "drake/examples/QPInverseDynamicsForHumanoids/generic_plan.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

class HumanoidManipPlan : public GenericHumanoidPlan {
 public:
  HumanoidManipPlan(const RigidBodyTree<double>& robot);

  // TODO: this is obviously wrong.
  void HandleManipPlan(const HumanoidStatus&rs);
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
