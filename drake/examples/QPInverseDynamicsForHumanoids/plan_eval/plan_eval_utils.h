#pragma once

#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * ContactState is intended to represent a set of bodies that are in contact.
 * Detailed information such as contact points, contact wrench are not included
 * here.
 */
typedef std::set<const RigidBody<double>*> ContactState;

/**
 * A wrapper class that stores a PiecewisePolynomial and its first and second
 * derivatives. This class is supposed to represent position, velocity and
 * acceleration. Thus, when the interpolating time is beyond the time bounds,
 * the interpolated velocity and acceleration will be set to zero, and the
 * interpolated position will peg at the terminal values. All dimensions are
 * assumed to be independent of each other.
 */
template <typename T>
class PiecewiseCubicTrajectory {
 public:
  PiecewiseCubicTrajectory() {}

  /**
   * Constructor.
   * @param position_traj PiecewisePolynomial that represents the position
   * trajectory. Its first and second derivatives are computed and stored.
   */
  PiecewiseCubicTrajectory(const PiecewisePolynomial<T>& position_traj) {
    q_ = position_traj;
    qd_ = q_.derivative();
    qdd_ = qd_.derivative();
  }

  /**
   * Returns the interpolated position at @p time.
   */
  VectorX<T> get_position(double time) const {
    return q_.value(time);
  }

  /**
   * Returns the interpolated velocity at @p time or zero if @p time is out of
   * the time bounds.
   */
  VectorX<T> get_velocity(double time) const {
    if (time <= q_.getStartTime() || time >= q_.getEndTime()) {
      return VectorX<T>::Zero(q_.rows());
    }
    return qd_.value(time);
  }

  /**
   * Returns the interpolated acceleration at @p time or zero if @p time is out
   * of the time bounds.
   */
  VectorX<T> get_acceleration(double time) const {
    if (time <= q_.getStartTime() || time >= q_.getEndTime()) {
      return VectorX<T>::Zero(q_.rows());
    }
    return qdd_.value(time);
  }

  /**
   * Returns the start time of this trajectory.
   */
  double get_start_time() const { return q_.getStartTime(); }

  /**
   * Returns the end time of this trajectory.
   */
  double get_end_time() const { return q_.getEndTime(); }

 private:
  PiecewisePolynomial<T> q_;
  PiecewisePolynomial<T> qd_;
  PiecewisePolynomial<T> qdd_;
};

/**
 * A wrapper class that represents a Cartesian trajectory, whose position part
 * is a PiecewisePolynomial, and the rotation part is a
 * PiecewiseQuaternionSlerp.
 */
template <typename T>
class PiecewiseCartesianTrajectory {
 public:
  /**
   * Constructs a PiecewiseCartesianTrajectory from given @p time and @p poses.
   * A cubic polynomial with zero end velocities is used to construct the
   * position part. There must be at least two elements in @p times and
   * @p poses.
   * @param times Breaks used to build the splines.
   * @param poses Knots used to build the splines.
   */
  static PiecewiseCartesianTrajectory<T> MakeCubicLinearWithZeroEndVelocity(
      const std::vector<T>& times,
      const std::vector<Isometry3<T>>& poses) {
    std::vector<MatrixX<T>> pos_knots(poses.size());
    eigen_aligned_std_vector<Matrix3<T>> rot_knots(poses.size());
    for (size_t i = 0; i < poses.size(); ++i) {
      pos_knots[i] = poses[i].translation();
      rot_knots[i] = poses[i].linear();
    }

    return PiecewiseCartesianTrajectory(
        PiecewisePolynomial<T>::Cubic(
            times, pos_knots, MatrixX<T>::Zero(3, 1), MatrixX<T>::Zero(3, 1)),
        PiecewiseQuaternionSlerp<T>(times, rot_knots));
  }

  /**
   * Constructor.
   * @param pos_traj PiecewisePolynomial that represents the position part.
   * @param rot_traj PiecewiseQuaternionSlerp that represents the rotation part.
   */
  PiecewiseCartesianTrajectory(const PiecewisePolynomial<T>& pos_traj,
                               const PiecewiseQuaternionSlerp<T>& rot_traj) {
    position_ = pos_traj;
    positiond_ = position_.derivative();
    positiondd_ = positiond_.derivative();
    orientation_ = rot_traj;
  }

  /**
   * Returns the interpolated pose at @p time.
   */
  Isometry3<T> get_pose(double time) const {
    Isometry3<T> pose;
    pose.translation() = position_.value(time);
    pose.linear() = orientation_.orientation(time).toRotationMatrix();
    return pose;
  }

  /**
   * Returns the interpolated velocity at @p time or zero if @p time is before
   * this trajectory's start time or after its end time.
   */
  Vector6<T> get_velocity(double time) const {
    if (time <= position_.getStartTime() || time >= position_.getEndTime()) {
      return Vector6<T>::Zero();
    }
    Vector6<T> velocity;
    velocity.template head<3>() = orientation_.angular_velocity(time);
    velocity.template tail<3>() = positiond_.value(time);
    return velocity;
  }

  /**
   * Returns the interpolated acceleration at @p time or zero if @p time is
   * before this trajectory's start time or after its end time.
   */
  Vector6<T> get_acceleration(double time) const {
    if (time <= position_.getStartTime() || time >= position_.getEndTime()) {
      return Vector6<T>::Zero();
    }
    Vector6<T> acceleration;
    acceleration.template head<3>() = orientation_.angular_acceleration(time);
    acceleration.template tail<3>() = positiondd_.value(time);
    return acceleration;
  }

  /**
   * Returns the start time of this trajectory.
   */
  double get_start_time() const { return position_.getStartTime(); }

  /**
   * Returns the end time of this trajectory.
   */
  double get_end_time() const { return position_.getEndTime(); }

 private:
  PiecewisePolynomial<T> position_;
  PiecewisePolynomial<T> positiond_;
  PiecewisePolynomial<T> positiondd_;

  PiecewiseQuaternionSlerp<T> orientation_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
