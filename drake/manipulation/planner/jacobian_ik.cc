#include "drake/manipulation/planner/jacobian_ik.h"

#include <memory>

#include "drake/common/text_logging.h"
#include "drake/multibody/constraint/rigid_body_constraint.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_ik.h"

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace manipulation {
namespace planner {

Vector6<double> JacobianIk::ComputePoseDiffInWorldFrame(
      const Isometry3<double>& pose0, const Isometry3<double>& pose1) {
  Vector6<double> diff = Vector6<double>::Zero();

  // Linear.
  diff.tail<3>() = (pose1.translation() - pose0.translation());

  // Angular.
  AngleAxis<double> rot_err(pose1.linear() * pose0.linear().transpose());
  diff.head<3>() = rot_err.axis() * rot_err.angle();

  return diff;
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
  v_lower_ = VectorX<double>::Constant(robot_->get_num_velocities(), -2);
  v_upper_ = VectorX<double>::Constant(robot_->get_num_velocities(), 2.);

  identity_ = MatrixX<double>::Identity(robot_->get_num_positions(),
                                        robot_->get_num_positions());
}

VectorX<double> JacobianIk::ComputeDofVelocity(const KinematicsCache<double>& cache0,
    const Vector6<double>& V_WE, const VectorX<double>& q_nominal,
    double dt) const {
  DRAKE_DEMAND(q_nominal.size() == robot_->get_num_positions());

  VectorX<double> ret;
  MatrixX<double> J = robot_->CalcBodySpatialVelocityJacobianInWorldFrame(
      cache0, *end_effector_);

  solvers::MathematicalProgram prog;
  solvers::VectorXDecisionVariable v =
    prog.NewContinuousVariables(robot_->get_num_velocities(), "v");

  // Add ee vel constraint
  solvers::QuadraticCost* cost =
      prog.AddL2NormCost(J, V_WE, v).constraint().get();

  // Add v constraint
  prog.AddBoundingBoxConstraint(v_lower_, v_upper_, v);

  // Add q upper and lower joint limit.
  prog.AddLinearConstraint(identity_ * dt,
      q_lower_ - cache0.getQ(), q_upper_ - cache0.getQ(), v);

  solvers::SolutionResult result = solver_.Solve(prog);
  DRAKE_DEMAND(result == solvers::SolutionResult::kSolutionFound);
  ret = prog.GetSolutionVectorValues();

  ////////////////

  // Constrain end effector speed to be J * ret.
  prog.AddLinearEqualityConstraint(J, J * ret, v);

  // Changed the cost to go towards the zero configuration.
  cost->UpdateCoefficients(identity_ * dt * dt,
                           (cache0.getQ() - q_nominal) * dt,
                           (cache0.getQ() - q_nominal).squaredNorm());

  result = solver_.Solve(prog);
  DRAKE_DEMAND(result == solvers::SolutionResult::kSolutionFound);
  ret = prog.GetSolutionVectorValues();

  return ret;
}

bool JacobianIk::Plan(const VectorX<double>& q0,
                      const std::vector<double>& times,
                      const std::vector<Isometry3<double>>& pose_traj,
                      const VectorX<double>& q_nominal,
                      std::vector<VectorX<double>>* q_sol) const {
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
    Vector6<double> V_WE_d = ComputePoseDiffInWorldFrame(pose_now, pose_traj[t]) / dt;

    v = ComputeDofVelocity(cache, V_WE_d, q_nominal, dt);

    q_now += v * dt;
    time_now = times[t];
    (*q_sol)[t] = q_now;
  }

  return false;
}

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
