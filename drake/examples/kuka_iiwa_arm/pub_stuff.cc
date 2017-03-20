#include <lcm/lcm-cpp.hpp>
#include "drake/lcmt_manipulator_plan_move_end_effector.hpp"
#include "drake/common/eigen_types.h"
#include "drake/util/lcmUtil.h"
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

int main() {
  lcm::LCM lcm;
  drake::lcmt_manipulator_plan_move_end_effector cmd;

  srand (time(NULL));
  cmd.timestamp = rand();

  cmd.num_steps = 3;
  cmd.utimes.resize(cmd.num_steps);
  cmd.poses.resize(cmd.num_steps);

  cmd.utimes[0] = 1e6;
  cmd.utimes[1] = 2e6;
  cmd.utimes[2] = 3e6;

  drake::Isometry3<double> pose(drake::Isometry3<double>::Identity());
  pose.translation() << 0.6, 0, 0.5;
  EncodePose(pose, cmd.poses[0]);

  pose.translation() << 0.6, 0.1, 0.3;
  EncodePose(pose, cmd.poses[1]);

  pose.translation() << 0.6, 0.2, 0.1;
  EncodePose(pose, cmd.poses[2]);

  lcm.publish("MOVL", &cmd);

  while(true);

  return 0;
}
