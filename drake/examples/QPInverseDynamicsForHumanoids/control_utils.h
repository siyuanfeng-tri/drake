#pragma once

#include <iostream>
#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/math/rotation_matrix.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_quaternion.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * This is used to compute target task space acceleration, which is the input
 * to the inverse dynamics controller.
 * The target acceleration is computed by:
 * acceleration_d = Kp*(x* - x) + Kd*(xd* - xd) + xdd*,
 * where x is pose, xd is velocity, and xdd is acceleration.
 * Variables with superscript * are the set points, and Kp and Kd are the
 * position and velocity gains.
 *
 * Pose "difference" is computed as:
 * H^w_d = E * H^w_m, E = H^w_d * H^w_m.inverse(), where
 * H^w_d = desired orientation in the world frame,
 * H^w_m = measured orientation in the world frame,
 * E = a small rotation in the world frame from measured to desired.
 *
 * The first terms 3 are angular accelerations, and the last 3 are linear
 * accelerations.
 */
template <typename Scalar>
class CartesianSetpoint {
 public:
  CartesianSetpoint();

  /**
   * @param pose_d Desired pose
   * @param vel_d Desired velocity
   * @param acc_d Desired feedforward acceleration
   * @param Kp Position gain
   * @param Kd Velocity gain
   */
  CartesianSetpoint(const Isometry3<Scalar>& pose_d,
                    const Vector6<Scalar>& vel_d, const Vector6<Scalar>& acc_d,
                    const Vector6<Scalar>& Kp, const Vector6<Scalar>& Kd);

  /**
   * Computes target acceleration using PD feedback + feedfoward acceleration.
   * @param pose Measured pose
   * @param vel Measured velocity
   * @return Computed spatial acceleration.
   */
  Vector6<Scalar> ComputeTargetAcceleration(const Isometry3<Scalar>& pose,
                                            const Vector6<Scalar>& vel) const;

  bool is_valid() const;

  // Getters
  inline const Isometry3<Scalar>& desired_pose() const { return pose_d_; }
  inline const Vector6<Scalar>& desired_velocity() const { return vel_d_; }
  inline const Vector6<Scalar>& desired_acceleration() const { return acc_d_; }
  inline const Vector6<Scalar>& Kp() const { return Kp_; }
  inline const Vector6<Scalar>& Kd() const { return Kd_; }

  // Setters
  inline Isometry3<Scalar>& mutable_desired_pose() { return pose_d_; }
  inline Vector6<Scalar>& mutable_desired_velocity() { return vel_d_; }
  inline Vector6<Scalar>& mutable_desired_acceleration() { return acc_d_; }
  inline Vector6<Scalar>& mutable_Kp() { return Kp_; }
  inline Vector6<Scalar>& mutable_Kd() { return Kd_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // Desired pose
  Isometry3<Scalar> pose_d_;
  // Desired velocity
  Vector6<Scalar> vel_d_;
  // Desired acceleration
  Vector6<Scalar> acc_d_;

  // Position gains
  Vector6<Scalar> Kp_;
  // Velocity gains
  Vector6<Scalar> Kd_;
};

template <typename Scalar>
inline std::ostream& operator<<(std::ostream& out,
                                const CartesianSetpoint<Scalar>& setpoint) {
  Vector3<Scalar> rpy = math::rotmat2rpy(setpoint.desired_pose().linear());
  out << "pose: (" << setpoint.desired_pose().translation().transpose()
      << "), (" << rpy.transpose() << ")" << std::endl;
  out << "velocity: " << setpoint.desired_velocity().transpose() << std::endl;
  out << "acceleration: " << setpoint.desired_acceleration().transpose()
      << std::endl;
  out << "Kp: " << setpoint.Kp().transpose() << std::endl;
  out << "Kd: " << setpoint.Kd().transpose() << std::endl;
  return out;
}

template <typename Scalar>
class VectorSetpoint {
 public:
  VectorSetpoint() {}

  explicit VectorSetpoint(int dim);

  /**
   * @param pos_d Desired position
   * @param vel_d Desired velocity
   * @param acc_d Desired feedforward acceleration
   * @param Kp Position gain
   * @param Kd Velocity gain
   */
  VectorSetpoint(const VectorX<Scalar>& pos_d, const VectorX<Scalar>& vel_d,
                 const VectorX<Scalar>& acc_d, const VectorX<Scalar>& Kp,
                 const VectorX<Scalar>& Kd);

  /**
   * Computes target acceleration using PD feedback + feedforward acceleration
   * @param idx Index
   * @param pos Measured position
   * @param vel Measured velocity
   * @return Computed acceleration
   */
  Scalar ComputeTargetAcceleration(int idx, Scalar pos, Scalar vel) const;

  /**
   * Computes target acceleration using PD feedback + feedforward acceleration
   * @param idx Index
   * @param pos Measured position
   * @param vel Measured velocity
   * @return Computed acceleration
   */
  VectorX<Scalar> ComputeTargetAcceleration(const VectorX<Scalar>& pos,
                                            const VectorX<Scalar>& vel) const;

  bool is_valid(int dim) const;

  // Getters
  inline const VectorX<Scalar>& desired_position() const { return pos_d_; }
  inline const VectorX<Scalar>& desired_velocity() const { return vel_d_; }
  inline const VectorX<Scalar>& desired_acceleration() const { return acc_d_; }
  inline const VectorX<Scalar>& Kp() const { return Kp_; }
  inline const VectorX<Scalar>& Kd() const { return Kd_; }
  inline int size() const { return pos_d_.size(); }

  // Setters
  inline VectorX<Scalar>& mutable_desired_position() { return pos_d_; }
  inline VectorX<Scalar>& mutable_desired_velocity() { return vel_d_; }
  inline VectorX<Scalar>& mutable_desired_acceleration() { return acc_d_; }
  inline VectorX<Scalar>& mutable_Kp() { return Kp_; }
  inline VectorX<Scalar>& mutable_Kd() { return Kd_; }

 private:
  // Desired position
  VectorX<Scalar> pos_d_;
  // Desired velocity
  VectorX<Scalar> vel_d_;
  // Desired acceleration
  VectorX<Scalar> acc_d_;

  // Position gains
  VectorX<Scalar> Kp_;
  // Velocity gains
  VectorX<Scalar> Kd_;
};

template <typename Scalar>
std::ostream& operator<<(std::ostream& out,
                         const VectorSetpoint<Scalar>& setpoint) {
  out << "pos: " << setpoint.desired_position().transpose() << std::endl;
  out << "vel: " << setpoint.desired_velocity().transpose() << std::endl;
  out << "acc: " << setpoint.desired_acceleration().transpose() << std::endl;
  out << "Kp: " << setpoint.Kp().transpose() << std::endl;
  out << "Kd: " << setpoint.Kd().transpose() << std::endl;
  return out;
}

template <typename Scalar>
class VectorTrajectoryTracker {
 public:
  VectorTrajectoryTracker() {}

  VectorTrajectoryTracker(
      const PiecewisePolynomial<Scalar>& traj,
      const VectorX<Scalar>& Kp, const VectorX<Scalar>& Kd);

  Scalar ComputeTargetAcceleration(int idx, double time, Scalar pos, Scalar vel) const;

  VectorX<Scalar> ComputeTargetAcceleration(double time,
                                            const VectorX<Scalar>& pos,
                                            const VectorX<Scalar>& vel) const;

 private:
  // TODO make these const
  mutable VectorSetpoint<Scalar> tracker_;
  PiecewisePolynomial<Scalar> traj_d_;
  PiecewisePolynomial<Scalar> trajd_d_;
  PiecewisePolynomial<Scalar> trajdd_d_;
};

template <typename Scalar>
class CartesianTrajectoryTracker {
 public:
  CartesianTrajectoryTracker() {}

  CartesianTrajectoryTracker(
      const PiecewisePolynomial<Scalar>& pos_traj,
      const PiecewiseQuaternionSlerp<Scalar>& rot_traj,
      const Vector6<Scalar>& Kp, const Vector6<Scalar>& Kd);

  Vector6<Scalar> ComputeTargetAcceleration(double time,
                                            const Isometry3<Scalar>& pose,
                                            const Vector6<Scalar>& vel) const;

 private:
  // TODO make these const
  mutable CartesianSetpoint<Scalar> tracker_;
  PiecewisePolynomial<Scalar> pos_traj_d_;
  PiecewisePolynomial<Scalar> pos_trajd_d_;
  PiecewisePolynomial<Scalar> pos_trajdd_d_;
  PiecewiseQuaternionSlerp<Scalar> rot_traj_d_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
