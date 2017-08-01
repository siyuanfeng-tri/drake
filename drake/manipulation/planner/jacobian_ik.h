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
             const Isometry3<double>& base_to_world);

  JacobianIk(const RigidBodyTree<double>* robot);

  bool Plan(const VectorX<double>& q0, const std::vector<double>& times,
            const std::vector<Isometry3<double>>& pose_traj,
            const RigidBodyFrame<double>& frame_E,
            const VectorX<double>& q_nominal,
            std::vector<VectorX<double>>* q_sol) const;

  /**
   * @param cache0 Captures the current state of the robot.
   * @param V_WE Desired end effector (frame E) velocity in the world frame.
   * @param dt Delta time.
   * @param gain_E Gain on V_WE_E specified in the end effector frame.
   * @return Resulting generalized velocity.
   */
  VectorX<double> ComputeDofVelocity(const KinematicsCache<double>& cache0,
      const RigidBodyFrame<double>& frame_E,
      const Vector6<double>& V_WE,
      const VectorX<double>& q_nominal,
      double dt,
      const Vector6<double>& gain_E = Vector6<double>::Constant(1)) const;

  /**
   * Returns constant reference to the robot model.
   */
  const RigidBodyTree<double>& get_robot() const { return *robot_; }

  void SetSamplingDt(double dt) { sampling_dt_ = dt; }

  double GetSamplingDt() const { return sampling_dt_; }

 private:
  std::unique_ptr<RigidBodyTree<double>> owned_robot_{nullptr};
  const RigidBodyTree<double>* robot_{nullptr};
  double sampling_dt_{5e-3};

  VectorX<double> q_lower_;
  VectorX<double> q_upper_;
  VectorX<double> v_lower_;
  VectorX<double> v_upper_;
  MatrixX<double> identity_;
  VectorX<double> zero_;

  mutable drake::solvers::GurobiSolver solver_;

  void Setup();
};

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
