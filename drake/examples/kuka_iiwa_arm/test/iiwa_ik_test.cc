#include "drake/examples/kuka_iiwa_arm/iiwa_ik_planner.h"

#include <fstream>
#include <string>
#include <unordered_map>

#include "drake/common/drake_path.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

void main() {
  const std::string kModelPath = GetDrakePath() +
      "/examples/kuka_iiwa_arm/urdf/"
      "iiwa14_simplified_collision.urdf";

  IiwaIkPlanner ik(kModelPath, "iiwa_link_ee", nullptr);
  std::vector<IiwaIkPlanner::IkCartesianWaypoint> waypoints(1);
  IiwaIkPlanner::IkResult result;
  bool status;

  Vector3<double> low(-0.5, -0.5, 0);
  Vector3<double> high(0.5, 0.5, 1);
  int xN = 2;
  int yN = 2;
  int zN = 2;
  double dx = (high(0) - low(0)) / xN;
  double dy = (high(1) - low(1)) / yN;
  double dz = (high(2) - low(2)) / zN;

  std::vector<std::vector<std::vector<std::pair<bool, VectorX<double>>>>> workspace;
  workspace.resize(xN);
  for (int xi = 0; xi < xN; xi++) {
    workspace[xi].resize(yN);
    for (int yi = 0; yi < yN; yi++) {
      workspace[xi][yi].resize(zN);
    }
  }

  VectorX<double> zero7 = VectorX<double>::Zero(7);

  std::ofstream ofs;
  ofs.open ("/home/sfeng/workspace.txt", std::ofstream::out);

  Vector3<double> pos_tol(0.005, 0.005, 0.005);
  double rot_tol = 0.05;

  for (int xi = 0; xi < xN; xi++) {
    for (int yi = 0; yi < yN; yi++) {
      for (int zi = 0; zi < zN; zi++) {
        double x = low(0) + xi * dx;
        double y = low(1) + yi * dy;
        double z = low(2) + zi * dz;
        waypoints[0].pose.translation() = Vector3<double>(x, y, z);
        std::cout << "pos: " << waypoints[0].pose.translation().transpose() << std::endl;
        status = ik.PlanTrajectory(waypoints, zero7, pos_tol, rot_tol, &result);

        if (status) {
          workspace[xi][yi][zi] = std::pair<bool, VectorX<double>>(true, result.q.col(1));
          std::cout << "success: " << waypoints[0].pose.translation().transpose() << std::endl;
        } else {
          workspace[xi][yi][zi] = std::pair<bool, VectorX<double>>(false, zero7);
          std::cout << "fail: " << waypoints[0].pose.translation().transpose() << std::endl;
        }

        ofs << waypoints[0].pose.translation().transpose() << " " << workspace[xi][yi][zi].second.transpose() << std::endl;
      }
    }
  }
  ofs.close();
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main() {
  drake::examples::kuka_iiwa_arm::main();
  return 0;
}
