#pragma once

#include <string>

#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/multibody/rigid_body_tree.h"
#include "robotlocomotion/robot_plan_t.hpp"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

class KukaIkPlanner {
 public:
  KukaIkPlanner(const std::string& model_path, const Isometry3<double>& base_to_world);
  KukaIkPlanner(const std::string& model_path, std::shared_ptr<RigidBodyFrame<double>> base_to_world);

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

  void SetEndEffector(const RigidBody<double>& end_effector_body) {
    end_effector_body_idx_ = end_effector_body.get_body_index();
  }

  void SetEndEffector(const std::string& link_name) {
    end_effector_body_idx_ = robot_->FindBodyIndex(link_name);
  }

  const RigidBodyTree<double>& get_robot() const { return *robot_; }

  bool PlanTrajectory(const std::vector<IkCartesianWaypoint>& waypoints, const VectorX<double>& q_current, IkResult* ik_res);

  robotlocomotion::robot_plan_t EncodeMessage(const IkResult& ik_res);

  static std::unique_ptr<PiecewisePolynomialTrajectory> GenerateFirstOrderHoldTrajectory(const IkResult& ik_res) {
    std::vector<MatrixX<double>> q(ik_res.q.cols());
    for (int i = 0; i < ik_res.q.cols(); ++i) {
      q[i] = ik_res.q.col(i);
    }
    return std::make_unique<PiecewisePolynomialTrajectory>(
        PiecewisePolynomial<double>::FirstOrderHold(ik_res.time, q));
  }

  std::unique_ptr<PiecewisePolynomialTrajectory> GenerateFirstOrderHoldTrajectoryFromCartesianWaypoints(
      const std::vector<double>& time_stamps,
      const std::vector<Vector3<double>>& way_point_list);


 private:
  bool PlanTrajectory(const std::vector<IkCartesianWaypoint>& waypoints, const VectorX<double>& q_current, const MatrixX<double>& q0, IkResult* ik_res, const Vector3<double>& pos_tol, double rot_tol);

  std::unique_ptr<RigidBodyTree<double>> robot_{nullptr};
  int end_effector_body_idx_;
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
