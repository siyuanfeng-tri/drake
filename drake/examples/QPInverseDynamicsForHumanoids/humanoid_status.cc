#include "humanoid_status.h"
#include <iostream>

const Vector3d HumanoidStatus::kFootToContactOffset = Vector3d(0, 0, -0.09);
const Vector3d HumanoidStatus::kFootToSensorOffset =
    Vector3d(0.0215646, 0.0, -0.051054);

void HumanoidStatus::FillKinematics(const std::shared_ptr<RigidBodyTree> robot,
                                    const RigidBody& body, Isometry3d* pose,
                                    Vector6d* vel, MatrixXd* J,
                                    Vector6d* Jdot_times_v,
                                    const Ref<const Vector3d>& local_offset) const {
  *pose = Isometry3d::Identity();
  pose->translation() = local_offset;
  *pose = robot->relativeTransform(cache_, 0, body.get_body_index()) * (*pose);

  *vel = GetTaskSpaceVel(*(robot), cache_, body, local_offset);
  *J = GetTaskSpaceJacobian(*(robot), cache_, body, local_offset);
  *Jdot_times_v =
      GetTaskSpaceJacobianDotTimesV(*(robot), cache_, body, local_offset);
}

Eigen::VectorXd HumanoidStatus::GetNominalPosition() const {
  Eigen::VectorXd q = Eigen::VectorXd::Zero(position_.size());

  q.segment<6>(0).setZero();
  q[joint_name_to_position_index().at("rightHipRoll")] = 0.01;
  q[joint_name_to_position_index().at("rightHipPitch")] = -0.5432;
  q[joint_name_to_position_index().at("rightKneePitch")] = 1.2195;
  q[joint_name_to_position_index().at("rightAnklePitch")] =
      -0.7070;
  q[joint_name_to_position_index().at("rightAnkleRoll")] = -0.0069;

  q[joint_name_to_position_index().at("leftHipRoll")] = -0.01;
  q[joint_name_to_position_index().at("leftHipPitch")] = -0.5432;
  q[joint_name_to_position_index().at("leftKneePitch")] = 1.2195;
  q[joint_name_to_position_index().at("leftAnklePitch")] = -0.7070;
  q[joint_name_to_position_index().at("leftAnkleRoll")] = 0.0069;

  q[joint_name_to_position_index().at("rightShoulderRoll")] = 1;
  q[joint_name_to_position_index().at("rightShoulderYaw")] = 0.5;
  q[joint_name_to_position_index().at("rightElbowPitch")] =
      M_PI / 2.;

  q[joint_name_to_position_index().at("leftShoulderRoll")] = -1;
  q[joint_name_to_position_index().at("leftShoulderYaw")] = 0.5;
  q[joint_name_to_position_index().at("leftElbowPitch")] =
      -M_PI / 2.;

  return q;
}

void HumanoidStatus::Update(double t, const Ref<const VectorXd>& q, const Ref<const VectorXd>& v,
                            const Ref<const VectorXd>& trq, const Ref<const Vector6d>& l_ft, const Ref<const Vector6d>& r_ft,
                            const Ref<const Matrix3d>& rot) {
  if (q.size() != position_.size() || v.size() != velocity_.size() ||
      trq.size() != joint_torque_.size()) {
    throw std::runtime_error("robot state update dimension mismatch.");
  }

  time_ = t;
  position_ = q;
  velocity_ = v;
  joint_torque_ = trq;

  cache_.initialize(position_, velocity_);
  robot_->doKinematics(cache_, true);

  M_ = robot_->massMatrix(cache_);
  eigen_aligned_unordered_map<RigidBody const*, drake::TwistVector<double>>
      f_ext;
  bias_term_ = robot_->dynamicsBiasTerm(cache_, f_ext);

  // com
  com_ = robot_->centerOfMass(cache_);
  J_com_ = robot_->centerOfMassJacobian(cache_);
  Jdot_times_v_com_ = robot_->centerOfMassJacobianDotTimesV(cache_);
  comd_ = J_com_ * v;

  // body parts
  FillKinematics(robot_, *pelv_.body, &pelv_.pose, &pelv_.vel, &pelv_.J,
                 &pelv_.Jdot_times_v);
  FillKinematics(robot_, *torso_.body, &torso_.pose, &torso_.vel, &torso_.J,
                 &torso_.Jdot_times_v);
  for (int s = 0; s < 2; s++) {
    FillKinematics(robot_, *foot_[s].body, &foot_[s].pose, &foot_[s].vel, &foot_[s].J,
                   &foot_[s].Jdot_times_v, kFootToContactOffset);
    FillKinematics(robot_, *foot_sensor_[s].body, &foot_sensor_[s].pose,
                   &foot_sensor_[s].vel, &foot_sensor_[s].J,
                   &foot_sensor_[s].Jdot_times_v, kFootToSensorOffset);
  }

  // ft sensor
  foot_wrench_in_sensor_frame_[Side::LEFT] = l_ft;
  foot_wrench_in_sensor_frame_[Side::RIGHT] = r_ft;
  for (int i = 0; i < 2; i++) {
    foot_wrench_in_sensor_frame_[i].head(3) =
        rot * foot_wrench_in_sensor_frame_[i].head(3);
    foot_wrench_in_sensor_frame_[i].tail(3) =
        rot * foot_wrench_in_sensor_frame_[i].tail(3);
    foot_wrench_in_world_frame_[i].head(3) =
        foot_sensor_[i].pose.linear() * foot_wrench_in_sensor_frame_[i].head(3);
    foot_wrench_in_world_frame_[i].tail(3) =
        foot_sensor_[i].pose.linear() * foot_wrench_in_sensor_frame_[i].tail(3);
  }

  // cop
  Vector2d cop_w[2];
  double Fz[2];
  for (int i = 0; i < 2; i++) {
    Fz[i] = foot_wrench_in_world_frame_[i][5];
    if (fabs(Fz[i]) < 1e-3) {
      cop_in_sensor_frame_[i][0] = 0;
      cop_in_sensor_frame_[i][1] = 0;
      cop_w[i][0] = foot_sensor_[i].pose.translation()[0];
      cop_w[i][1] = foot_sensor_[i].pose.translation()[1];
    } else {
      // cop relative to the ft sensor
      cop_in_sensor_frame_[i][0] =
          -foot_wrench_in_sensor_frame_[i][1] / foot_wrench_in_sensor_frame_[i][5];
      cop_in_sensor_frame_[i][1] =
          foot_wrench_in_sensor_frame_[i][0] / foot_wrench_in_sensor_frame_[i][5];

      cop_w[i][0] = -foot_wrench_in_world_frame_[i][1] / Fz[i] +
                    foot_sensor_[i].pose.translation()[0];
      cop_w[i][1] = foot_wrench_in_world_frame_[i][0] / Fz[i] +
                    foot_sensor_[i].pose.translation()[1];
    }
  }

  // This is assuming that both feet are on the same horizontal surface.
  cop_ = (cop_w[Side::LEFT] * Fz[Side::LEFT] +
          cop_w[Side::RIGHT] * Fz[Side::RIGHT]) /
         (Fz[Side::LEFT] + Fz[Side::RIGHT]);
}
