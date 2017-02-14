#pragma once

#include <string>

#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/multibody/rigid_body_tree.h"
#include "robotlocomotion/robot_plan_t.hpp"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

class IiwaIkPlanner {
 public:
  IiwaIkPlanner(const std::string& model_path,
                const std::string& end_effector_link_name,
                const Isometry3<double>& base_to_world);
  IiwaIkPlanner(const std::string& model_path,
                const std::string& end_effector_link_name,
                std::shared_ptr<RigidBodyFrame<double>> base_to_world);

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

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  void SetEndEffector(const RigidBody<double>& end_effector_body) {
    end_effector_body_idx_ = end_effector_body.get_body_index();
  }

  void SetEndEffector(const std::string& link_name) {
    end_effector_body_idx_ = robot_->FindBodyIndex(link_name);
  }

  const RigidBodyTree<double>& get_robot() const { return *robot_; }

  bool PlanTrajectory(const std::vector<IkCartesianWaypoint>& waypoints,
                      const VectorX<double>& q_current,
                      const Vector3<double>& position_tol, double rotation_tol,
                      IkResult* ik_res);

  robotlocomotion::robot_plan_t EncodeMessage(const IkResult& ik_res);

  static std::unique_ptr<PiecewisePolynomialTrajectory>
  GenerateFirstOrderHoldTrajectory(const IkResult& ik_res) {
    std::vector<MatrixX<double>> q(ik_res.q.cols());
    for (int i = 0; i < ik_res.q.cols(); ++i) {
      q[i] = ik_res.q.col(i);
    }
    return std::make_unique<PiecewisePolynomialTrajectory>(
        PiecewisePolynomial<double>::FirstOrderHold(ik_res.time, q));
  }

  std::unique_ptr<PiecewisePolynomialTrajectory>
  GenerateFirstOrderHoldTrajectoryFromCartesianWaypoints(
      const std::vector<double>& time_stamps,
      const std::vector<Vector3<double>>& way_point_list,
      const Vector3<double>& position_tol = Vector3<double>(0.005, 0.005,
                                                            0.005),
      double rotation_tol = 0.05);

 private:
  bool SolveIk(const IkCartesianWaypoint& waypoint,
               const VectorX<double>& q_current,
               const Vector3<double>& position_tol, double rot_tolerance,
               VectorX<double>* ik_res);

  std::unique_ptr<RigidBodyTree<double>> robot_{nullptr};
  int end_effector_body_idx_;
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
