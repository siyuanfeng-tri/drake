#pragma once

#include <memory>
#include <random>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/solvers/gurobi_solver.h"

namespace drake {
namespace manipulation {
namespace planner {

class JacobianIk {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JacobianIk)

  /**
   * Linear = pose1.translation - pos0.translation
   * Angular: R_err = pose1.linear() * pose0.linear().transpose().
   */
  static Vector6<double> ComputePoseDiffInWorldFrame(
      const Isometry3<double>& pose0, const Isometry3<double>& pose1);

  /**
   * Assumes that q.size() == v.size().
   */
  JacobianIk(const std::string& model_path,
             const std::string& end_effector_link_name,
             const Isometry3<double>& base_to_world);

  bool Plan(const VectorX<double>& q0, const std::vector<double>& times,
            const std::vector<Isometry3<double>>& pose_traj,
            const VectorX<double>& q_nominal,
            std::vector<VectorX<double>>* q_sol) const;

  /**
   * This function solves two quadratic programs to find a generalized
   * velocity v s.t.
   * 1) the resulting end effector velocity matches @p V_WE as closely as
   * possible.
   * 2) Without sacrificing 1), use redundancy in the robot to tracking a
   * nominal configuration.
   * @param cache0 Captures the current state of the robot.
   * @param V_WE Desired end effector (frame E) velocity in the world frame.
   * @param dt Delta time.
   * @return Resulting generalized velocity.
   */
  VectorX<double> ComputeDofVelocity(const KinematicsCache<double>& cache0,
      const Vector6<double>& V_WE, const VectorX<double>& q_nominal,
      double dt) const;

  /**
   * Sets end effector to @p end_effector_body.
   */
  void SetEndEffector(const RigidBody<double>& end_effector_body) {
    end_effector_ = &end_effector_body;
    end_effector_body_idx_ = end_effector_body.get_body_index();
  }

  /**
   * Sets end effector to @p link_name.
   */
  void SetEndEffector(const std::string& link_name) {
    end_effector_ = robot_->FindBody(link_name);
    end_effector_body_idx_ = end_effector_->get_body_index();
  }

  /**
   * Returns constant reference to the robot model.
   */
  const RigidBodyTree<double>& get_robot() const { return *robot_; }

  void SetSamplingDt(double dt) { sampling_dt_ = dt; }

  double GetSamplingDt() const { return sampling_dt_; }

 private:
  std::unique_ptr<RigidBodyTree<double>> robot_{nullptr};
  const RigidBody<double>* end_effector_{nullptr};
  int end_effector_body_idx_{};
  double sampling_dt_{5e-3};

  VectorX<double> q_lower_;
  VectorX<double> q_upper_;
  VectorX<double> v_lower_;
  VectorX<double> v_upper_;
  MatrixX<double> identity_;

  mutable drake::solvers::GurobiSolver solver_;
};

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
