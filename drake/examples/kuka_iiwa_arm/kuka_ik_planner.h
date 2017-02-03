#pragma once

#include <string>
#include "drake/multibody/rigid_body_tree.h"
#include "drake/common/eigen_types.h"
#include "robotlocomotion/robot_plan_t.hpp"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

class KukaIkPlanner {
 public:
  KukaIkPlanner(const std::string& model_path);

  struct IkResult {
    std::vector<double> time;
    std::vector<int> info;
    MatrixX<double> q;
  };

  struct IkCartesianWaypoint {
    double time;
    Isometry3<double> pose;
    bool enforce_quat;

    IkCartesianWaypoint() {
      time = 0;
      pose.setIdentity();
      enforce_quat = false;
    }
  };

  bool PlanTrajectory(const std::vector<IkCartesianWaypoint>& waypoints, const VectorX<double>& q0, IkResult* ik_res);
  robotlocomotion::robot_plan_t EncodeMessage(const IkResult& ik_res);

 private:
  std::unique_ptr<RigidBodyTree<double>> robot_;
  int end_effector_body_idx_;
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
