#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/manipulator_plan.h"

#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"

#include <vector>

#include "drake/common/unused.h"
#include "drake/manipulation/util/motion_plan_translator.h"
#include "drake/util/lcmUtil.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

using manipulation::PiecewiseCartesianTrajectory;
using manipulation::PiecewiseCubicTrajectory;
using manipulation::CartesianTrajectoryTranslator;
using manipulation::DofTrajectoryTranslator;

template <typename T>
void ManipulatorPlan<T>::InitializeGenericPlanDerived(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) {
  unused(paramset);  // TODO(jwnimmer-tri) This seems bad.
}

template <typename T>
void ManipulatorPlan<T>::HandlePlanMessageGenericPlanDerived(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
    const void* message_bytes, int message_length) {
  unused(paramset);  // TODO(jwnimmer-tri) This seems bad.

  /////////////////////////////////////////////////////////////
  // HACK
  // This first version assumes the given plan is actually a cartesian move
  // plan without contacts.

  // Tries to decode as a lcmt_motion_plan message.
  lcmt_motion_plan msg;
  int consumed = msg.decode(message_bytes, 0, message_length);
  DRAKE_DEMAND(consumed == message_length);

  // Q traj.
  PiecewiseCubicTrajectory<double> dof_traj;
  DofTrajectoryTranslator::DecodeMessage(msg.dof_motion, &dof_traj);
  dof_traj.shift_right(robot_status.time());
  this->set_dof_trajectory(dof_traj);

  // Body traj.
  std::string name;
  std::cout << "asdf: " << msg.body_motions.size() << std::endl;
  PiecewiseCartesianTrajectory<double> traj;
  for (const auto& body_motion : msg.body_motions) {
    CartesianTrajectoryTranslator::DecodeMessage(body_motion, &name, &traj);
    std::cout << name << std::endl;
    const RigidBody<T>* body = robot_status.robot().FindBody(name);
    traj.shift_right(robot_status.time());
    this->set_body_trajectory(body, traj);
  }

  // Contacts.
  override_contacts_.clear();
  ContactInformation tmp(*robot_status.robot().FindBody("world")); // will get reset
  if (msg.num_contact_states != 0) {
    // TODO: currently, we are only looking at the first thing.
    const lcmt_contact_state& contact_state = msg.contact_states.front();
    for (int i = 0; i < contact_state.num_bodies_in_contact; ++i) {
      DecodeContactInformation(robot_status.robot(),
          contact_state.bodies_in_contact[i], &tmp);
      override_contacts_.emplace(tmp.body_name(), tmp);
    }
  }
}

template <typename T>
void ManipulatorPlan<T>::ModifyPlanGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet&,
      const param_parsers::RigidBodyTreeAliasGroups<T>&) {
  if (!override_contacts_.empty()) {
    override_contacts_.at("iiwa_link_ee").mutable_desired_force()[1] = 2 * cos(robot_status.time());
    override_contacts_.at("iiwa_link_ee").mutable_desired_force()[2] = 2 * sin(robot_status.time());
  }
}

template <typename T>
void ManipulatorPlan<T>::MakeDebugMessage(
    const HumanoidStatus& robot_stauts,
    const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
    std::vector<uint8_t>* raw_bytes) const {

}

template <typename T>
GenericPlan<T>* ManipulatorPlan<T>::CloneGenericPlanDerived()
    const {
  return new ManipulatorPlan(*this);
}

template class ManipulatorPlan<double>;

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
