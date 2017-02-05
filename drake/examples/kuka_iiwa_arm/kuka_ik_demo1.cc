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
  std::vector<KukaIkPlanner::IkCartesianWaypoint> waypoints;

  DRAKE_DEMAND(argc == 2);
  int move = atoi(argv[1]);

  KukaIkPlanner::IkCartesianWaypoint wp;

  int N = 3;
  double z_high = 0.3;
  double z_low = 0.1;
  double dz = (z_high - z_low) / N;
  double dt = 0.3;

  // approach
  wp.time = 1;
  wp.pose.translation() << 0.68, 0, z_high;
  wp.enforce_quat = true;

  switch (move) {
    case 0:
      waypoints.push_back(wp);
      break;
    case 1:
      // down
      wp.pose.translation()[2] = z_high;
      for (int i = 0; i < N; i++) {
        wp.time += dt;
        wp.pose.translation()[2] -= dz;
        waypoints.push_back(wp);
      }
      break;

    case 2:
      // up
      wp.pose.translation()[2] = z_low;
      for (int i = 0; i < N; i++) {
        wp.time += dt;
        wp.pose.translation()[2] += dz;
        waypoints.push_back(wp);
      }
      break;

    case 3:
      // approach other
      wp.time += 1;
      wp.pose.translation() << 0, 0.68, z_high;
      wp.pose.linear() = Matrix3<double>(AngleAxis<double>(M_PI / 2., Vector3<double>::UnitZ()));
      waypoints.push_back(wp);
      break;

    case 4:
      // down
      wp.pose.translation() << 0, 0.68, z_high;
      wp.pose.linear() = Matrix3<double>(AngleAxis<double>(M_PI / 2., Vector3<double>::UnitZ()));
      for (int i = 0; i < N; i++) {
        wp.time += dt;
        wp.pose.translation()[2] -= dz;
        waypoints.push_back(wp);
      }
      break;

    case 5:
      // up
      wp.pose.translation() << 0, 0.68, z_low;
      wp.pose.linear() = Matrix3<double>(AngleAxis<double>(M_PI / 2., Vector3<double>::UnitZ()));
      for (int i = 0; i < N; i++) {
        wp.time += dt;
        wp.pose.translation()[2] += dz;
        waypoints.push_back(wp);
      }
      break;
  }

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
