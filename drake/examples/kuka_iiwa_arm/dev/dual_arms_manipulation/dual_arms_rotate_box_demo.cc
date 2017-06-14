#include "drake/examples/kuka_iiwa_arm/dev/dual_arms_manipulation/dual_arms_rotate_box_planner.h"

#include <iostream>
#include <fstream>

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
int DoMain() {
  auto tree = ConstructDualArmAndBox();

  Eigen::Matrix<double, 6, 1> q_box = Eigen::Matrix<double, 6, 1>::Zero();
  VisualizePosture(tree.get(), Eigen::Matrix<double, 7, 1>::Zero(), Eigen::Matrix<double, 7, 1>::Zero(), q_box);
  return 0;
}
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main() {
  return drake::examples::kuka_iiwa_arm::DoMain();
}
