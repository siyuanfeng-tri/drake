#include "drake/examples/kuka_iiwa_arm/jjz_common.h"

namespace drake {
namespace examples {
namespace jjz {

const Matrix3<double> R_ET(
    AngleAxis<double>(M_PI, Vector3<double>::UnitY()).toRotationMatrix());
const Isometry3<double> X_ET(
    Eigen::Translation<double, 3>(Vector3<double>(0, 0, 0.15)) *
    Isometry3<double>(R_ET));
const Isometry3<double> X_WG(
    Eigen::Translation<double, 3>(Vector3<double>(0.5, 0.2, 0.)) *
    AngleAxis<double>(-M_PI / 2., Vector3<double>::UnitZ()));
const std::string kEEName("iiwa_link_7");

IiwaState::IiwaState(const RigidBodyTree<double>& iiwa)
    : iiwa_(iiwa),
      end_effector_(*iiwa_.FindBody(kEEName)),
      cache_(iiwa.CreateKinematicsCache()),
      q_(VectorX<double>::Zero(iiwa.get_num_positions())),
      v_(VectorX<double>::Zero(iiwa.get_num_velocities())),
      trq_(VectorX<double>::Zero(iiwa.get_num_actuators())),
      ext_trq_(VectorX<double>::Zero(iiwa.get_num_actuators())) {
  DRAKE_DEMAND(iiwa.get_num_positions() == iiwa.get_num_velocities());
  DRAKE_DEMAND(iiwa.get_num_actuators() == iiwa.get_num_velocities());
}

bool IiwaState::UpdateState(const lcmt_iiwa_status& msg) {
  // Check msg.
  DRAKE_DEMAND(msg.num_joints == iiwa_.get_num_positions());

  const double cur_time = msg.utime / 1e6;
  // Same time stamp, should just return.
  if (init_ && cur_time == time_) return false;

  // Do velocity update first.
  if (init_) {
    // TODO need to filter.
    delta_time_ = cur_time - time_;
    for (int i = 0; i < msg.num_joints; ++i) {
      v_[i] = (msg.joint_position_measured[i] - q_[i]) / delta_time_;
    }
  } else {
    delta_time_ = 0;
    v_.setZero();
  }

  // Update time, position, and torque.
  time_ = cur_time;
  for (int i = 0; i < msg.num_joints; ++i) {
    q_[i] = msg.joint_position_measured[i];
    trq_[i] = msg.joint_torque_measured[i];
    ext_trq_[i] = msg.joint_torque_external[i];
  }

  // Update kinematics.
  cache_.initialize(q_, v_);
  iiwa_.doKinematics(cache_);

  J_ = iiwa_.CalcBodySpatialVelocityJacobianInWorldFrame(cache_, end_effector_);
  ext_wrench_ = J_.transpose().colPivHouseholderQr().solve(ext_trq_);

  init_ = true;

  return true;
}

}  // namespace jjz
}  // namespace examples
}  // namespace drake
