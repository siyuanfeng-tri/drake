#pragma once

#include "humanoid_status.h"
#include <iostream>
#include <fstream>

#include "drake/solvers/optimization.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/gurobi_solver.h"

class CartesianSetPoint {
 public:
  int frame_id;

  Isometry3d pose_d;
  Vector6d vel_d;
  Vector6d acc_d;

  Vector6d Kp;
  Vector6d Kd;
  bool disable_mask[6];

  CartesianSetPoint() {
    frame_id = 0;
    pose_d.setIdentity();
    vel_d.setZero();
    acc_d.setZero();
    Kp.setZero();
    Kd.setZero();
    for (int i = 0; i < 6; i++)
      disable_mask[i] = false;
  }

  CartesianSetPoint(int fid, const Isometry3d &p_d, const Vector6d &v_d, const Vector6d &vd_d, const Vector6d &Kp, const Vector6d &Kd) {
    frame_id = fid;
    pose_d = p_d;
    vel_d = v_d;
    acc_d = vd_d;
    this->Kp = Kp;
    this->Kd = Kd;
    for (int i = 0; i < 6; i++)
      disable_mask[i] = false;
  }

  // rotation first, position second
  Vector6d ComputeAccelerationTarget(const Isometry3d &pose, const Vector6d &vel) const {
    // feedforward acc_d + velocity feedback
    Vector6d qdd = acc_d;
    qdd += (Kd.array() * (vel_d - vel).array()).matrix();

    // pose feedback
    Matrix3d R_err = pose_d.linear() * pose.linear().transpose();
    AngleAxisd angle_axis_err(R_err);

    Vector3d pos_err = pose_d.translation() - pose.translation();
    Vector3d rot_err = angle_axis_err.axis() * angle_axis_err.angle();

    // orientation
    qdd.segment<3>(0) += (Kp.segment<3>(0).array() * rot_err.array()).matrix();

    // position
    qdd.segment<3>(3) += (Kp.segment<3>(3).array() * pos_err.array()).matrix();

    return qdd;
  }
};

class SupportElement {
 private:
  const RigidBody& body_;
  // these are local offsets within body_
  std::vector<Eigen::Vector3d> contact_points_;
  // this is also in body frame, assuming normal is the same for all contact points
  Eigen::Vector3d normal_;
  double mu_;

 public:
  SupportElement(const RigidBody &b) : body_(b) {
    normal_ = Vector3d(0, 0, 1);
    mu_ = 1;
  }

  Eigen::MatrixXd ComputeBasisMatrix(const RigidBodyTree &robot, const KinematicsCache<double> &cache, int num_basis_per_contact_pt) const {
    Eigen::MatrixXd basis(3 * contact_points_.size(), num_basis_per_contact_pt  * contact_points_.size());
    Eigen::Matrix3d body_rot = robot.relativeTransform(cache, 0, body_.get_body_index()).linear();

    Eigen::Vector3d t1, t2, tangent_vec, base;
    double theta;

    basis.setZero();
    if (fabs(1 - normal_[2]) < EPSILON) {  // handle the unit-normal case (since it's
      // unit length, just check z)
      t1 << 1, 0, 0;
    } else if (fabs(1 + normal_[2]) < EPSILON) {
      t1 << -1, 0, 0;  // same for the reflected case
    } else {           // now the general case
      t1 << normal_[1], -normal_[0], 0;
      t1 /= sqrt(normal_[1] * normal_[1] + normal_[0] * normal_[0]);
    }
    t2 = t1.cross(normal_);
    
    for (size_t i = 0; i < contact_points_.size(); i++) {
      for (int k = 0; k < num_basis_per_contact_pt; k++) {
        theta = k * 2 * M_PI / num_basis_per_contact_pt;
        tangent_vec = cos(theta) * t1 + sin(theta) * t2;
        base = (normal_ + mu_ * tangent_vec).normalized();
        basis.block(3 * i, num_basis_per_contact_pt * i + k, 3, 1) = body_rot * base;
      }
    }
    return basis;
  }

  Eigen::MatrixXd ComputeJacobianAtContactPoints(const RigidBodyTree &robot, const KinematicsCache<double> &cache) const {
    Eigen::MatrixXd JJ(3 * contact_points_.size(), robot.number_of_velocities());
    for (size_t i = 0; i < contact_points_.size(); i++)
      JJ.block(3 * i, 0, 3, robot.number_of_velocities()) = GetTaskSpaceJacobian(robot, cache, body_, contact_points_[i]).bottomRows(3);
    return JJ;
  }

  inline std::vector<Eigen::Vector3d> &get_mutable_contact_points() { return contact_points_; }
  inline double mu() const { return mu_; }
  inline void set_mu(double m) { mu_ = m; }
  inline const std::vector<Eigen::Vector3d> &contact_points() const { return contact_points_; }
  inline const Eigen::Vector3d &normal() const { return normal_; }
  inline const RigidBody& body() const { return body_; }
};
 

/**
 * Input to the QP inverse dynamics controller
 */
struct QPInput {
  // Names for each generalized coordinate.
  std::vector<std::string> coord_names;

  std::vector<SupportElement> all_contacts;

  // Desired task space accelerations for various body parts.
  // Postfix _d indicates desired values.
  Vector3d comdd_d;
  Vector6d pelvdd_d;
  Vector6d torsodd_d;
  Vector6d footdd_d[2];

  VectorXd vd_d;         ///< Desired generalized coordinate accelerations
  Vector6d wrench_d[2];  ///< Desired contact wrench

  // These are weights for each cost term.
  // Prefix w_ indicates weights.
  double w_com;
  double w_pelv;
  double w_torso;
  double w_foot;
  double w_vd;
  double w_wrench_reg;
};

// temporary stuff
void InitQPInput(const RigidBodyTree& r, QPInput* input);
void PrintQPInput(const QPInput &input);
inline bool is_qp_input_sane(const QPInput& input) {
  return ((int)input.coord_names.size() == input.vd_d.size()) &&
         (input.coord_names.size() != 0);
}

/**
 * Output of the QP inverse dynamics controller
 */
struct QPOutput {
  // Names for each generalized coordinate.
  std::vector<std::string> coord_names;

  // Computed task space accelerations of various body parts.
  Vector3d comdd;
  Vector6d pelvdd;
  Vector6d torsodd;
  Vector6d footdd[2];

  VectorXd vd;            ///< Computed generalized coordinate accelerations
  VectorXd joint_torque;  ///< Computed joint torque

  /// Computed contact wrench in the world frame
  Vector6d foot_wrench_in_world_frame[2];
  /// Computed contact wrench transformed to the sensor frame
  Vector6d foot_wrench_in_sensor_frame[2];
};

void InitQPOutput(const RigidBodyTree& r, QPOutput* output);
void PrintQPOutput(const QPOutput& output);
double ComputeQPCost(const HumanoidStatus& rs, const QPInput& input,
                     const QPOutput& output);
inline bool is_qp_output_sane(const QPOutput& output) {
  bool ret = (int)output.coord_names.size() == output.vd.size();
  ret &= output.vd.size() == output.joint_torque.size() + 6;
  return ret;
}

/**
 * Parameter for the QP inverse dynamics controller.
 */
struct QPParam {
  double mu;     // Friction approx for tangential force. Fx, Fy < |mu * Fz|
  double mu_Mz;  // Friction approx for normal torque. Mz < |mu * Mz|
  double x_max;  // Size of the foot in the x direction, in the foot frame
  double x_min;
  double y_max;  // Size of the foot in the y direction, in the foot frame
  double y_min;
};




class QPController {
 public:
  QPParam param;

  /**
   * The current version explicitly uses SNOPT, but it is really solving a
   * quadratic program. It also instantiates
   * an OptimizationProblem every call. It should call a QP solver and maintain
   * a persistent "workspace" for the optimization problem in the future.
   *
   * @return 0 if success, < if error.
   */
  int Control(const HumanoidStatus& rs, const QPInput& input, QPOutput* output);

  void SetupDoubleSupport(const HumanoidStatus& rs);
  void SetupSingleSupport(const HumanoidStatus& rs);

  explicit QPController(const HumanoidStatus& rs) {
    param.mu = 1;
    param.mu_Mz = 0.1;
    param.x_max = 0.2;
    param.x_min = -0.05;
    param.y_max = 0.05;
    param.y_min = -0.05;
    SetupDoubleSupport(rs);
  }

 private:
  // These are temporary matrices used by the controller.
  Eigen::MatrixXd torque_linear_;
  Eigen::VectorXd torque_constant_;
  Eigen::MatrixXd dynamics_linear_;
  Eigen::VectorXd dynamics_constant_;

  Eigen::MatrixXd inequality_linear_;
  Eigen::VectorXd inequality_upper_bound_;
  Eigen::VectorXd inequality_lower_bound_;

  // update the problem
  drake::solvers::OptimizationProblem prog_;
  drake::solvers::SnoptSolver solver_;
  //GurobiSolver solver_;

  int num_contacts_;
  int num_vd_;
  int num_wrench_;
  int num_torque_;
  int num_variable_;

  void ResizeQP();

  std::shared_ptr<drake::solvers::LinearEqualityConstraint> eq_dynamics_;
  std::vector<std::shared_ptr<drake::solvers::LinearEqualityConstraint>> eq_contacts_;
  std::shared_ptr<drake::solvers::LinearConstraint> ineq_contact_wrench_;
  std::shared_ptr<drake::solvers::LinearConstraint> ineq_torque_limit_;
  std::shared_ptr<drake::solvers::QuadraticConstraint> cost_comdd_;
  std::shared_ptr<drake::solvers::QuadraticConstraint> cost_pelvdd_;
  std::shared_ptr<drake::solvers::QuadraticConstraint> cost_torsodd_;
  std::shared_ptr<drake::solvers::QuadraticConstraint> cost_vd_reg_;
  std::shared_ptr<drake::solvers::QuadraticConstraint> cost_lambda_reg_;
};


class QPControllerNew {
 private:
  // These are temporary matrices used by the controller.
  Eigen::MatrixXd stacked_contact_jacobians_;
  Eigen::MatrixXd basis_to_force_matrix_;

  Eigen::MatrixXd torque_linear_;
  Eigen::VectorXd torque_constant_;
  Eigen::MatrixXd dynamics_linear_;
  Eigen::VectorXd dynamics_constant_;

  Eigen::MatrixXd inequality_linear_;
  Eigen::VectorXd inequality_upper_bound_;
  Eigen::VectorXd inequality_lower_bound_;

  Eigen::MatrixXd JB_;
  Eigen::MatrixXd point_force_to_wrench_;
  Eigen::VectorXd contact_wrenches_;

  // update the problem
  drake::solvers::OptimizationProblem prog_;
  drake::solvers::SnoptSolver solver_;
  //GurobiSolver solver_;

  int num_contacts_;
  int num_vd_;
  int num_point_forces_;
  int num_basis_;
  int num_torque_;
  int num_variable_;

  void ResizeQP(const HumanoidStatus& rs, const std::vector<SupportElement> &all_contacts);

  std::shared_ptr<drake::solvers::LinearEqualityConstraint> eq_dynamics_;
  std::vector<std::shared_ptr<drake::solvers::LinearEqualityConstraint>> eq_contacts_;
  std::shared_ptr<drake::solvers::LinearConstraint> ineq_contact_wrench_;
  std::shared_ptr<drake::solvers::LinearConstraint> ineq_torque_limit_;
  std::shared_ptr<drake::solvers::QuadraticConstraint> cost_comdd_;
  std::shared_ptr<drake::solvers::QuadraticConstraint> cost_pelvdd_;
  std::shared_ptr<drake::solvers::QuadraticConstraint> cost_torsodd_;
  std::shared_ptr<drake::solvers::QuadraticConstraint> cost_vd_reg_;

  void SetZero() {
    stacked_contact_jacobians_.setZero();
    basis_to_force_matrix_.setZero();
    torque_linear_.setZero();
    torque_constant_.setZero();
    dynamics_linear_.setZero();
    dynamics_constant_.setZero();
    inequality_linear_.setZero();
    inequality_upper_bound_.setZero();
    inequality_lower_bound_.setZero();

    JB_.setZero();
    point_force_to_wrench_.setZero();
  }

 public:
  QPParam param;

  int Control(const HumanoidStatus& rs, const QPInput& input, QPOutput* output);

  const int n_basis_per_contact_point;
  void AllocateProblem();

  explicit QPControllerNew(const HumanoidStatus& rs, int n) : n_basis_per_contact_point(n) {
    param.mu = 1;
    param.mu_Mz = 0.1;
    param.x_max = 0.2;
    param.x_min = -0.05;
    param.y_max = 0.05;
    param.y_min = -0.05;
  }
};
