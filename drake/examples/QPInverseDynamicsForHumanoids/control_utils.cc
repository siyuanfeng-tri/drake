#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

template <typename Scalar>
CartesianSetpoint<Scalar>::CartesianSetpoint() {
  pose_d_.setIdentity();
  vel_d_.setZero();
  acc_d_.setZero();
  Kp_.setZero();
  Kd_.setZero();
}

template <typename Scalar>
CartesianSetpoint<Scalar>::CartesianSetpoint(
    const Isometry3<Scalar>& pose_d,
    const Vector6<Scalar>& vel_d, const Vector6<Scalar>& acc_d,
    const Vector6<Scalar>& Kp, const Vector6<Scalar>& Kd) {
  pose_d_ = pose_d;
  vel_d_ = vel_d;
  acc_d_ = acc_d;
  Kp_ = Kp;
  Kd_ = Kd;
}

template <typename Scalar>
Vector6<Scalar> CartesianSetpoint<Scalar>::ComputeTargetAcceleration(
    const Isometry3<Scalar>& pose,
    const Vector6<Scalar>& vel) const {
  // feedforward acc + velocity feedback
  Vector6<Scalar> acc = acc_d_;
  acc += (Kd_.array() * (vel_d_ - vel).array()).matrix();

  // pose feedback
  // H^w_d = desired orientation in the world frame,
  // H^w_m = measured orientation in the world frame,
  // E = a small rotation in the world frame from measured to desired.
  // H^w_d = E * H^w_m, E = H^w_d * H^w_m.inverse()
  Quaternion<Scalar> quat_d(pose_d_.linear());
  Quaternion<Scalar> quat(pose.linear());
  // Make sure the relative rotation between the desired and the measured
  // rotation goes the "shortest" way.
  if (quat_d.dot(quat) < 0) {
    quat.w() *= -1;
    quat.x() *= -1;
    quat.y() *= -1;
    quat.z() *= -1;
  }
  AngleAxis<Scalar> angle_axis_err(quat_d * quat.inverse().normalized());

  Vector3<Scalar> pos_err = pose_d_.translation() - pose.translation();
  Vector3<Scalar> rot_err = angle_axis_err.axis() * angle_axis_err.angle();

  // orientation
  acc.template head<3>() +=
    (Kp_.template head<3>().array() * rot_err.array()).matrix();

  // position
  acc.template tail<3>() +=
    (Kp_.template tail<3>().array() * pos_err.array()).matrix();

  return acc;
}

template <typename Scalar>
bool CartesianSetpoint<Scalar>::is_valid() const {
  bool ret = pose_d_.translation().allFinite();
  ret &= Matrix3<Scalar>::Identity().isApprox(
      pose_d_.linear() * pose_d_.linear().transpose(),
      Eigen::NumTraits<Scalar>::epsilon());
  ret &= vel_d_.allFinite();
  ret &= acc_d_.allFinite();
  ret &= Kp_.allFinite();
  ret &= Kd_.allFinite();
  return ret;
}




template <typename Scalar> VectorSetpoint<Scalar>::VectorSetpoint(int dim) {
  pos_d_ = VectorX<Scalar>::Zero(dim);
  vel_d_ = VectorX<Scalar>::Zero(dim);
  acc_d_ = VectorX<Scalar>::Zero(dim);
  Kp_ = VectorX<Scalar>::Zero(dim);
  Kd_ = VectorX<Scalar>::Zero(dim);
}

template <typename Scalar> VectorSetpoint<Scalar>::VectorSetpoint(
    const VectorX<Scalar>& pos_d, const VectorX<Scalar>& vel_d,
    const VectorX<Scalar>& acc_d, const VectorX<Scalar>& Kp,
    const VectorX<Scalar>& Kd) {
  if (pos_d.size() != vel_d.size() || pos_d.size() != acc_d.size() ||
      pos_d.size() != Kp.size() || pos_d.size() != Kd.size()) {
    throw std::runtime_error(
        "Setpoints and gains have different dimensions.");
  }
  pos_d_ = pos_d;
  vel_d_ = vel_d;
  acc_d_ = acc_d;
  Kp_ = Kp;
  Kd_ = Kd;
}

template <typename Scalar>
Scalar VectorSetpoint<Scalar>::ComputeTargetAcceleration(int idx, Scalar pos,
    Scalar vel) const {
  if (idx < 0 || idx >= pos_d_.size())
    throw std::runtime_error("Index out of bound.");
  Scalar acc = acc_d_[idx];
  acc += Kp_[idx] * (pos_d_[idx] - pos);
  acc += Kd_[idx] * (vel_d_[idx] - vel);
  return acc;
}

template <typename Scalar>
VectorX<Scalar> VectorSetpoint<Scalar>::ComputeTargetAcceleration(
    const VectorX<Scalar>& pos,
    const VectorX<Scalar>& vel) const {
  if (pos.size() != vel.size() || pos.size() != pos_d_.size()) {
    throw std::runtime_error(
        "Setpoints and states have different dimensions.");
  }
  VectorX<Scalar> acc(pos_d_.size());
  for (int i = 0; i < size(); ++i)
    acc[i] = ComputeTargetAcceleration(i, pos[i], vel[i]);
  return acc;
}

template <typename Scalar>
bool VectorSetpoint<Scalar>::is_valid(int dim) const {
  bool ret = pos_d_.size() == vel_d_.size() &&
    pos_d_.size() == acc_d_.size() && pos_d_.size() == Kp_.size() &&
    pos_d_.size() == Kd_.size() && pos_d_.size() == dim;
  ret &= pos_d_.allFinite();
  ret &= vel_d_.allFinite();
  ret &= acc_d_.allFinite();
  ret &= Kp_.allFinite();
  ret &= Kd_.allFinite();
  return ret;
}



template <typename Scalar>
VectorTrajectoryTracker<Scalar>::VectorTrajectoryTracker(
    const PiecewisePolynomial<Scalar>& traj,
    const VectorX<Scalar>& Kp, const VectorX<Scalar>& Kd) {
  DRAKE_DEMAND(traj.rows() >= 1);
  DRAKE_DEMAND(traj.cols() == 1);
  DRAKE_DEMAND(traj.rows() == Kp.size());
  DRAKE_DEMAND(traj.rows() == Kd.size());

  tracker_ = VectorSetpoint<Scalar>(traj.rows());
  traj_d_ = traj;
  trajd_d_ = traj_d_.derivative();
  trajdd_d_ = trajd_d_.derivative();
  tracker_.mutable_Kp() = Kp;
  tracker_.mutable_Kd() = Kd;
}

template <typename Scalar>
Scalar VectorTrajectoryTracker<Scalar>::ComputeTargetAcceleration(
    int idx, double time, Scalar pos, Scalar vel) const {
  throw std::runtime_error("NOT IMPLEMENTED");
  return 0;
}

template <typename Scalar>
VectorX<Scalar> VectorTrajectoryTracker<Scalar>::ComputeTargetAcceleration(
    double time,
    const VectorX<Scalar>& pos,
    const VectorX<Scalar>& vel) const {
  tracker_.mutable_desired_position() = traj_d_.value(time);
  tracker_.mutable_desired_velocity() = trajd_d_.value(time);
  tracker_.mutable_desired_acceleration() = trajdd_d_.value(time);

  return tracker_.ComputeTargetAcceleration(pos, vel);
}


template <typename Scalar>
CartesianTrajectoryTracker<Scalar>::CartesianTrajectoryTracker(
    const PiecewisePolynomial<Scalar>& pos_traj,
    const PiecewiseQuaternionSlerp<Scalar>& rot_traj,
    const Vector6<Scalar>& Kp, const Vector6<Scalar>& Kd) {
  DRAKE_DEMAND(pos_traj.rows() == 3);
  DRAKE_DEMAND(pos_traj.cols() == 1);

  pos_traj_d_ = pos_traj;
  pos_trajd_d_ = pos_traj_d_.derivative();
  pos_trajdd_d_ = pos_trajd_d_.derivative();

  rot_traj_d_ = rot_traj;

  tracker_.mutable_Kp() = Kp;
  tracker_.mutable_Kd() = Kd;
}

template <typename Scalar>
Vector6<Scalar> CartesianTrajectoryTracker<Scalar>::ComputeTargetAcceleration(
    double time,
    const Isometry3<Scalar>& pose,
    const Vector6<Scalar>& vel) const {
  // Position
  tracker_.mutable_desired_pose().linear() =
      Matrix3<Scalar>(rot_traj_d_.orientation(time));
  tracker_.mutable_desired_pose().translation() =
      pos_traj_d_.value(time);

  // Velocity
  tracker_.mutable_desired_velocity().template head<3>() = rot_traj_d_.angular_velocity(time);
  tracker_.mutable_desired_velocity().template tail<3>() = pos_trajd_d_.value(time);

  // Acceleration
  tracker_.mutable_desired_acceleration().template head<3>() = rot_traj_d_.angular_acceleration(time);
  tracker_.mutable_desired_acceleration().template tail<3>() = pos_trajdd_d_.value(time);

  return tracker_.ComputeTargetAcceleration(pose, vel);
}

template class CartesianSetpoint<double>;
template class VectorSetpoint<double>;
template class VectorTrajectoryTracker<double>;
template class CartesianTrajectoryTracker<double>;

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
