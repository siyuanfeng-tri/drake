#include "drake/examples/QPInverseDynamicsForHumanoids/manip_plan.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

HumanoidManipPlan::HumanoidManipPlan(const RigidBodyTree<double>& robot)
    : GenericHumanoidPlan(robot) {
  tracked_bodies_.insert(pelvis_);
  tracked_bodies_.insert(torso_);
}

void HumanoidManipPlan::HandleManipPlan(const HumanoidStatus&rs) {
  std::vector<double> time({0, 1});
  int num_T = static_cast<int>(time.size());
  std::vector<Eigen::MatrixXd> q_d(num_T, rs.position());
  //std::vector<Eigen::MatrixXd> v_d(num_T, rs.velocity());
  std::vector<Eigen::MatrixXd> com_d(num_T);

  // q traj
  PiecewisePolynomial<double> q_traj = PiecewisePolynomial<double>::ZeroOrderHold(time, q_d);
  dof_tracker_ = VectorTrajectoryTracker<double>(q_traj, Kp_joints_, Kd_joints_);

  // com and body knot points
  std::vector<std::vector<MatrixX<double>>> pos_d(tracked_bodies_.size(), std::vector<MatrixX<double>>(num_T, MatrixX<double>::Zero(3,1)));
  std::vector<eigen_aligned_std_vector<Quaternion<double>>> rot_d(tracked_bodies_.size(), eigen_aligned_std_vector<Quaternion<double>>(num_T));

  VectorX<double> q, v;
  Isometry3<double> pose;
  Vector6<double> vel;
  for (int t = 0; t < num_T; t++) {
    // TODO fix this
    q = q_d[t];
    //v = v_d[t];
    KinematicsCache<double> cache_plan = robot_->doKinematics(q);

    com_d[t] = robot_->centerOfMass(cache_plan).segment<2>(0);

    int b = 0;
    for (const RigidBody<double>* body : tracked_bodies_) {
      pose = robot_->relativeTransform(cache_plan, 0, body->get_body_index());
      pos_d[b][t] = pose.translation();
      rot_d[b][t] = Quaternion<double>(pose.linear());
      b++;
    }
  }

  // CoM
  PiecewisePolynomial<double> zmp_d = PiecewisePolynomial<double>::FirstOrderHold(time, com_d);
  Vector4<double> xcom0;
  xcom0 << rs.com().head<2>(), rs.comd().head<2>();
  zmp_planner_.Plan(zmp_d, xcom0, com_height_);

  // Body trajs
  PiecewisePolynomial<double> body_pos_traj;
  PiecewiseQuaternionSlerp<double> body_rot_traj;
  int b = 0;
  for (const RigidBody<double>* body : tracked_bodies_) {
    body_pos_traj = PiecewisePolynomial<double>::FirstOrderHold(time, pos_d[b]);
    body_rot_traj = PiecewiseQuaternionSlerp<double>(time, rot_d[b]);

    body_trackers_.emplace(body, CartesianTrajectoryTracker<double>(body_pos_traj, body_rot_traj, Kp_pelvis_, Kd_pelvis_));
    b++;
  }

  // Contacts
  std::vector<ContactState> contacts(1);
  contacts[0] = double_support();
  contacts_traj_ = PiecewiseContactInformation(time, contacts);

  // Interp timer.
  interp_t0_ = rs.time();
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
