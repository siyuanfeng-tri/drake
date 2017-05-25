#include "drake/manipulation/util/motion_plan_translator.h"

namespace drake {
namespace manipulation {

void MotionPlanTranslator::InitializeMessage(lcmt_motion_plan* msg) {
  msg->timestamp = 0;

  msg->num_contact_states = 0;
  msg->contact_states.clear();

  DofTrajectoryTranslator::InitializeMessage(&(msg->dof_motion));

  msg->num_body_motions = 0;
  msg->body_motions.clear();
}

}  // namespace manipulation
}  // namespace drake
