#include "drake/examples/PlanarPushing/dubins_interface.h"

#include <iostream>

int main() {
  // Limit surface A_11.
  double ls_a = 1.1;
  // Limit surface A_33 / rho^2
  double ls_b = 4500;
  // Coefficient of contact friction
  double mu = 0.5;
  Eigen::Vector2d pt;
  pt << 0, -0.02;
  Eigen::Vector2d normal;
  normal << 0, 1;
  DubinsPushPlanner planner(pt, normal, mu, ls_a, ls_b);

  Eigen::Vector3d start_pose;
  start_pose << 0, 0, 0;
  Eigen::Vector3d goal_pose;
  goal_pose << 0, 0, M_PI / 2.0;
  int num_way_points = 100;
  Eigen::Matrix<double, Eigen::Dynamic, 3> object_poses;
  Eigen::Matrix<double, Eigen::Dynamic, 3> pusher_poses;
  planner.PlanPath(start_pose, goal_pose, num_way_points, &object_poses,
                   &pusher_poses);
  for (int i = 0; i < num_way_points; ++i) {
    std::cout << object_poses.row(i) << std::endl;
    std::cout << pusher_poses.row(i) << std::endl;
  }
}
