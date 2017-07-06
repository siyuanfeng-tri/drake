#include "drake/manipulation/planner/jacobian_ik.h"

#include <memory>

#include "drake/common/text_logging.h"
#include "drake/multibody/constraint/rigid_body_constraint.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_ik.h"

#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace manipulation {
namespace planner {

Vector6<double> compute_velocity(const Isometry3<double>& pose0,
                                 const Isometry3<double>& pose1, double dt) {
  Vector6<double> vel = Vector6<double>::Zero();

  // Linear.
  vel.tail<3>() = (pose1.translation() - pose0.translation()) / dt;

  // Angular.
  AngleAxis<double> rot_err(pose1.linear() * pose0.linear().transpose());
  vel.head<3>() = rot_err.axis() * rot_err.angle() / dt;

  return vel;
}

JacobianIk::JacobianIk(const std::string& model_path,
                       const std::string& end_effector_link_name,
                       const Isometry3<double>& base_to_world) {
  auto base_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "world", nullptr,
      base_to_world);

  robot_ = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFile(
      model_path, multibody::joints::kFixed, base_frame, robot_.get());
  DRAKE_DEMAND(robot_->get_num_positions() == robot_->get_num_velocities());

  SetEndEffector(end_effector_link_name);

  q_lower_ = robot_->joint_limit_min;
  q_upper_ = robot_->joint_limit_max;
}

VectorX<double> JacobianIk::solve_v(const KinematicsCache<double>& cache0,
                                    const Vector6<double>& xd, double dt) {
  std::cout << "q: " << cache0.getQ().transpose() << "\n";

  std::cout << xd.transpose() << "\n";

  solvers::MathematicalProgram prog;
  solvers::VectorXDecisionVariable v =
      prog.NewContinuousVariables(robot_->get_num_velocities(), "v");

  // Add ee vel constraint
  /*
  prog.AddLinearEqualityConstraint(
      robot_->CalcBodySpatialVelocityJacobianInWorldFrame(cache0,
                                                          *end_effector_),
      xd, v);
  */
  prog.AddL2NormCost(
      robot_->CalcBodySpatialVelocityJacobianInWorldFrame(cache0,
                                                          *end_effector_),
      xd, v);

  std::cout << q_lower_.transpose() << "\n";
  std::cout << q_upper_.transpose() << "\n";

  prog.AddBoundingBoxConstraint(
      -2 * VectorX<double>::Ones(robot_->get_num_velocities()),
       2 * VectorX<double>::Ones(robot_->get_num_velocities()),
       v);

  /*
  // Add q upper and lower joint limit.
  prog.AddLinearConstraint(
      MatrixX<double>::Identity(robot_->get_num_positions(),
                                robot_->get_num_positions()) * dt,
      q_lower_ - cache0.getQ(), q_upper_ - cache0.getQ(), v);

  // Add a normalization term
  prog.AddL2NormCost(
      MatrixX<double>::Identity(robot_->get_num_positions(),
                                robot_->get_num_positions()) * dt,
      -cache0.getQ(), v);
  */

  solvers::SolutionResult result = prog.Solve();
  DRAKE_DEMAND(result == solvers::SolutionResult::kSolutionFound);
  VectorX<double> ret = prog.GetSolutionVectorValues();

  //std::cout << "lb:" << (ret * dt + cache0.getQ() - q_lower_).transpose() << "\n";
  //std::cout << "ub:" << (ret * dt + cache0.getQ() - q_upper_).transpose() << "\n";

  return ret;
}

bool JacobianIk::Plan(const VectorX<double>& q0,
                      const std::vector<double>& times,
                      const std::vector<Isometry3<double>>& pose_traj,
                      std::vector<VectorX<double>>* q_sol) {
  DRAKE_DEMAND(times.size() == pose_traj.size());

  KinematicsCache<double> cache = robot_->CreateKinematicsCache();
  VectorX<double> q_now = q0;
  Isometry3<double> pose_now;
  double time_now = 0;

  VectorX<double> v;

  q_sol->resize(pose_traj.size());

  for (size_t t = 0; t < pose_traj.size(); ++t) {
    cache.initialize(q_now);
    robot_->doKinematics(cache);

    pose_now = robot_->CalcBodyPoseInWorldFrame(cache, *end_effector_);
    double dt = times[t] - time_now;
    Vector6<double> xd_d = compute_velocity(pose_now, pose_traj[t], dt);

    v = solve_v(cache, xd_d, dt);

    q_now += v * dt;
    time_now = times[t];
    (*q_sol)[t] = q_now;
  }

  return false;
}

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
