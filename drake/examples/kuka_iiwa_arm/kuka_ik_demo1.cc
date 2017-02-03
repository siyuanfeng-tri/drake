/// @file
///
/// Generates a canned IK demo plan for an iiwa arm starting from the
/// zero configuration and sends that plan over lcm using the
/// robot_plan_t message.

#include <iostream>
#include <memory>

#include <lcm/lcm-cpp.hpp>

#include "drake/examples/kuka_iiwa_arm/kuka_ik_planner.h"
#include "drake/common/drake_path.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";

int main(int argc, const char* argv[]) {
  std::string path = GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf";
  KukaIkPlanner planner(path);

  KukaIkPlanner::IkResult ik_res;
  std::vector<KukaIkPlanner::IkCartesianWaypoint> waypoints(2);
  waypoints[0].time = 1;
  waypoints[0].pose.translation() << 0.8, 0, 0.1;

  waypoints[1].time = 2;
  waypoints[1].pose.translation() << 0, 0.8, 0.1;

  planner.PlanTrajectory(waypoints, VectorX<double>::Zero(7), &ik_res);

  robotlocomotion::robot_plan_t plan = planner.EncodeMessage(ik_res);
  lcm::LCM lcm;
  return lcm.publish(kLcmPlanChannel, &plan);
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake


int main(int argc, const char* argv[]) {
  return drake::examples::kuka_iiwa_arm::main(argc, argv);
}
