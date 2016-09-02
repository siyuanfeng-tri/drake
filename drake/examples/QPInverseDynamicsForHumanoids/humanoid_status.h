#pragma once

#include "drake/systems/robotInterfaces/Side.h"
#include "rigid_body_tree_utils.h"

using namespace Eigen;

/**
 * A handy struct that stores important kinematic properties.
 * For all the velocity / acceleration / wrench, the first 3 are always angular,
 * and the last 3 are linear.
 */
class BodyOfInterest {
 private:
  /// Name of the BodyOfInterest
  std::string name_;
  /// The link which this BOI is attached to
  const RigidBody& body_;
  /// Offset is specified in the body frame.
  Vector3d offset_;

  Isometry3d pose_;
  /// This is the task space velocity, or twist of a frame that has the same
  /// orientation as the world frame, but located at the origin of the body
  /// frame.
  Vector6d vel_;
  /// Task space Jacobian, xdot = J * v
  MatrixXd J_;
  /// Task space Jd * v
  Vector6d Jdot_times_v_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  explicit BodyOfInterest(const std::string& name, const RigidBody& body,
                          const Vector3d& off)
      : name_(name), body_(body), offset_(off) {}

  /**
   * Updates pose, velocity, Jacobian, Jacobian_dot_times_v based on @p robot
   * and @p cache.
   * @param robot is the robot model.
   * @param cache is the kinematics cache. It needs to be initialed first
   */
  void Update(const RigidBodyTree& robot,
              const KinematicsCache<double>& cache) {
    pose_.translation() = offset_;
    pose_.linear().setIdentity();
    pose_ = robot.relativeTransform(cache, 0, body_.get_body_index()) * pose_;

    vel_ = GetTaskSpaceVel(robot, cache, body_, offset_);
    J_ = GetTaskSpaceJacobian(robot, cache, body_, offset_);
    Jdot_times_v_ = GetTaskSpaceJacobianDotTimesV(robot, cache, body_, offset_);
  }

  inline const std::string& name() const { return name_; }
  inline const RigidBody& body() const { return body_; }
  inline const Isometry3d& pose() const { return pose_; }
  inline const Vector6d& velocity() const { return vel_; }
  inline const MatrixXd& J() const { return J_; }
  inline const Vector6d& Jdot_times_v() const { return Jdot_times_v_; }
};

/**
 * Mostly a thin wrapper on RigidBodyTree.
 * It has kinematic values such as task space velocity of various body parts,
 * some measured contact force / torque information, joint torque, etc.
 */
class HumanoidStatus {
 public:
  /// Offset from the foot frame to contact position in the foot frame.
  static const Vector3d kFootToContactOffset;
  /// Offset from the foot frame to force torque sensor in the foot frame.
  static const Vector3d kFootToSensorOffset;

  explicit HumanoidStatus(const std::shared_ptr<RigidBodyTree> robot_in)
      : robot_(robot_in),
        cache_(robot_->bodies),
        bodies_of_interest_{
            BodyOfInterest("pelvis", *robot_->FindBody("pelvis"),
                           Vector3d::Zero()),
            BodyOfInterest("torso", *robot_->FindBody("torso"),
                           Vector3d::Zero()),
            BodyOfInterest("leftFoot", *robot_->FindBody("leftFoot"),
                           Vector3d::Zero()),
            BodyOfInterest("rightFoot", *robot_->FindBody("rightFoot"),
                           Vector3d::Zero()),
            BodyOfInterest("leftFootSensor", *robot_->FindBody("leftFoot"),
                           kFootToSensorOffset),
            BodyOfInterest("rightFootSensor", *robot_->FindBody("rightFoot"),
                           kFootToSensorOffset),
        } {
    // Build map
    body_name_to_id_ = std::unordered_map<std::string, int>();
    for (auto it = robot_->bodies.begin(); it != robot_->bodies.end(); ++it) {
      body_name_to_id_[(*it)->get_name()] = it - robot_->bodies.begin();
    }

    joint_name_to_position_index_ = std::unordered_map<std::string, int>();
    for (int i = 0; i < robot_->number_of_positions(); i++) {
      joint_name_to_position_index_[robot_->getPositionName(i)] = i;
    }
    for (size_t i = 0; i < robot_->actuators.size(); i++) {
      actuator_name_to_id_[robot_->actuators[i].name_] = i;
    }

    time_ = time0_ = 0;

    position_.resize(robot_->number_of_positions());
    velocity_.resize(robot_->number_of_velocities());
    joint_torque_.resize(robot_->actuators.size());
  }

  /**
   * Do kinematics and compute useful information based on kinematics and
   * measured force torque information.
   * @param time is in seconds
   * @param q is the vector or generalized positions.
   * @param v is the vector of generalized velocities.
   * @param trq is joint torque, should be in the same order as @p v, not
   * in robot->actuators order
   * @param l_ft is wrench measured at the foot force torque sensor
   * location.
   * @param r_ft is wrench measured at the foot force torque sensor
   * location.
   * @param rot rotates @p l_ft and @p r_ft in the same orientation as
   * the foot frame. This is useful if the foot ft sensor has a different
   * orientation than the foot.
   */
  void Update(double t, const Ref<const VectorXd>& q,
              const Ref<const VectorXd>& v, const Ref<const VectorXd>& trq,
              const Ref<const Vector6d>& l_ft, const Ref<const Vector6d>& r_ft,
              const Ref<const Matrix3d>& rot = Matrix3d::Identity());

  /**
   * Returns a nominal q.
   */
  Eigen::VectorXd GetNominalPosition() const;

  inline const RigidBodyTree& robot() const { return *robot_; }
  inline const KinematicsCache<double>& cache() const { return cache_; }
  inline const std::unordered_map<std::string, int>& body_name_to_id() const {
    return body_name_to_id_;
  }
  inline const std::unordered_map<std::string, int>&
  joint_name_to_position_index() const {
    return joint_name_to_position_index_;
  }
  inline const std::unordered_map<std::string, int>& actuator_name_to_id()
      const {
    return actuator_name_to_id_;
  }

  inline double time() const { return time_; }
  inline const VectorXd& position() const { return position_; }
  inline const VectorXd& velocity() const { return velocity_; }
  inline const VectorXd& joint_torque() const { return joint_torque_; }
  inline const MatrixXd& M() const { return M_; }
  inline const VectorXd& bias_term() const { return bias_term_; }
  inline const Vector3d& com() const { return com_; }
  inline const Vector3d& comd() const { return comd_; }
  inline const MatrixXd& J_com() const { return J_com_; }
  inline const Vector3d& Jdot_times_v_com() const { return Jdot_times_v_com_; }
  inline const MatrixXd& centroidal_momentum_matrix() const {
    return centroidal_momentum_matrix_;
  }
  inline const Vector6d& centroidal_momentum_matrix_dot_times_v() const {
    return centroidal_momentum_matrix_dot_times_v_;
  }
  inline const BodyOfInterest& pelv() const { return bodies_of_interest_[0]; }
  inline const BodyOfInterest& torso() const { return bodies_of_interest_[1]; }
  inline const BodyOfInterest& foot(Side::SideEnum s) const {
    if (s == Side::LEFT)
      return bodies_of_interest_[2];
    else
      return bodies_of_interest_[3];
  }
  inline const BodyOfInterest& foot(int s) const {
    return foot(Side::values.at(s));
  }
  inline const BodyOfInterest& foot_sensor(Side::SideEnum s) const {
    if (s == Side::LEFT)
      return bodies_of_interest_[4];
    else
      return bodies_of_interest_[5];
  }
  inline const Vector2d& cop() const { return cop_; }
  inline const Vector2d& cop_in_sensor_frame(Side::SideEnum s) const {
    return cop_in_sensor_frame_[s];
  }
  inline const Vector6d& foot_wrench_in_sensor_frame(Side::SideEnum s) const {
    return foot_wrench_in_sensor_frame_[s];
  }
  inline const Vector6d& foot_wrench_in_world_frame(Side::SideEnum s) const {
    return foot_wrench_in_world_frame_[s];
  }

  inline const BodyOfInterest& foot_sensor(int s) const {
    return foot_sensor(Side::values.at(s));
  }
  inline const Vector2d& cop_in_sensor_frame(int s) const {
    return cop_in_sensor_frame(Side::values.at(s));
  }
  inline const Vector6d& foot_wrench_in_sensor_frame(int s) const {
    return foot_wrench_in_sensor_frame(Side::values.at(s));
  }
  inline const Vector6d& foot_wrench_in_world_frame(int s) const {
    return foot_wrench_in_world_frame(Side::values.at(s));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  const std::shared_ptr<RigidBodyTree> robot_;
  KinematicsCache<double> cache_;

  /// Maps body name to its index
  std::unordered_map<std::string, int> body_name_to_id_;
  /// Maps joint name to its index
  std::unordered_map<std::string, int> joint_name_to_position_index_;
  /// Maps actuator name to its index
  std::unordered_map<std::string, int> actuator_name_to_id_;

  double time0_;
  double time_;

  // Pos and Vel include 6 dof for the floating base.
  VectorXd position_;  /// Position in generalized coordinate
  VectorXd velocity_;  /// Velocity in generalized coordinate
  // In the same order as vel, but trq contains only actuated joints.
  VectorXd joint_torque_;  /// Joint torque

  MatrixXd M_;          ///< Inertial matrix
  VectorXd bias_term_;  ///< Bias term: M * vd + h = tau + J^T * lambda

  // Computed from kinematics
  Vector3d com_;               ///< Center of mass
  Vector3d comd_;              ///< Com velocity
  MatrixXd J_com_;             ///< Com Jacobian: comd = J_com * v
  Vector3d Jdot_times_v_com_;  ///< J_com_dot * v
  // Centroidal momentum = [angular; linear] momentum.
  // [angular; linear] = centroidal_momentum_matrix_ * v
  MatrixXd centroidal_momentum_matrix_;
  Vector6d centroidal_momentum_matrix_dot_times_v_;

  // A list of body of interest, e.g. pelvis, feet, etc.
  std::vector<BodyOfInterest> bodies_of_interest_;

  Vector2d cop_;  ///< Center of pressure
  Vector2d
      cop_in_sensor_frame_[2];  ///< Individual center of pressure in foot frame

  Vector6d foot_wrench_in_sensor_frame_[2];  ///< Wrench rotated to align with
                                             ///the foot frame, located at the
                                             ///sensor position.
  Vector6d foot_wrench_in_world_frame_[2];   ///< Wrench rotated to align with
                                             ///the world frame, located at the
                                             ///ankle joint.
};
