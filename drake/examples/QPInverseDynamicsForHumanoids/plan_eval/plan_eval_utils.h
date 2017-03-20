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

template <typename T>
T average_angles(T a, T b) {
  T x = fmod(fabs(a - b), 2 * M_PI);
  if (x >= 0 && x <= M_PI)
    return fmod((a + b) / 2., 2 * M_PI);
  else if (x >= M_PI && x < 1.5 * M_PI)
    return fmod((a + b) / 2., 2 * M_PI) + M_PI;
  else
    return fmod((a + b) / 2., 2 * M_PI) - M_PI;
}

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
   * A cubic polynomial is used to construct the position part. There must be
   * at least two elements in @p times and @p poses.
   * If @p zero_end_point_linear_velocities is true, zero velocities are set
   * for the linear velocity. When @p zero_end_point_linear_velocities is
   * false and there are only two elements in @p time, a linear spline is used
   * to represent the position part. PiecewiseQuaternionSlerp is used for the
   * orientation part.
   * @param times Breaks used to build the splines.
   * @param poses Knots used to build the splines.
   * @param zero_end_point_linear_velocities If true, the start and end
   * velocity for the position spline are set to zero.
   */
  PiecewiseCartesianTrajectory(const std::vector<T>& times,
                               const std::vector<Isometry3<T>>& poses,
                               bool zero_end_point_linear_velocities) {
    std::vector<MatrixX<T>> pos_knots(poses.size());
    eigen_aligned_std_vector<Matrix3<T>> rot_knots(poses.size());
    for (size_t i = 0; i < poses.size(); ++i) {
      pos_knots[i] = poses[i].translation();
      rot_knots[i] = poses[i].linear();
    }

    if (zero_end_point_linear_velocities) {
      position_ = PiecewisePolynomial<T>::Cubic(
          times, pos_knots, MatrixX<T>::Zero(3, 1), MatrixX<T>::Zero(3, 1));
    } else {
      if (pos_knots.size() >= 3) {
        position_ = PiecewisePolynomial<T>::Cubic(times, pos_knots);
      } else {
        position_ = PiecewisePolynomial<T>::FirstOrderHold(times, pos_knots);
      }
    }

    positiond_ = position_.derivative();
    positiondd_ = positiond_.derivative();

    orientation_ = PiecewiseQuaternionSlerp<T>(times, rot_knots);
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
