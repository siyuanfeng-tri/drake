#include "drake/examples/QPInverseDynamicsForHumanoids/walking_plan.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

static double get_yaw(const Matrix3<double>& rot) {
  Vector2<double> x_axis = rot.col(0).head<2>();
  x_axis.normalize();
  return std::atan2(x_axis(1), x_axis(0));
}

HumanoidWalkingPlan::HumanoidWalkingPlan(const RigidBodyTree<double>& robot)
    : GenericHumanoidPlan(robot) {
  tracked_bodies_.insert(pelvis_);
  tracked_bodies_.insert(torso_);
}

void HumanoidWalkingPlan::HandleWalkingPlan(const HumanoidStatus& rs) {
  cur_state_ = WEIGHT_TRANSFER;

  // make up a dummy fs
  double y = 0.1;
  for (int i = 0; i < 5; ++i) {
    struct footstep_t footstep;
    footstep.is_right_foot = (i % 2 == 0);
    if (footstep.is_right_foot) {
      y = -0.1;
    } else {
      y = 0.1;
    }
    footstep.pose.linear().setIdentity();
    footstep.pose.translation() << 0, y, 0.09;

    footstep_plan_.push_back(footstep);
  }

  interp_t0_ = rs.time();
  GenerateTrajs(0, rs.position(), rs.velocity(), double_support());
  SwitchContact(0);
}

bool HumanoidWalkingPlan::DoStateTransition(const HumanoidStatus& rs) {
  double cur_time = rs.time();
  double plan_time = cur_time - interp_t0_;

  const ContactState& planned_cs = contacts_traj_.get_contacts(plan_time);
  const footstep_t &cur_step = footstep_plan_.front();
  const RigidBody<double>* swing_foot = cur_step.is_right_foot ? right_foot_ : left_foot_;

  switch (cur_state_) {
    case WEIGHT_TRANSFER:
      if (plan_time >= planned_contact_switch_time_) {
        // replace the swing up segment with a new one that starts from the current foot pose.
        KinematicsCache<double> cache = robot_->doKinematics(rs.position(), rs.velocity());
        Eigen::Isometry3d swing_foot0 = robot_->relativeTransform(cache, 0, swing_foot->get_body_index());

        Isometry3<double> swing_touchdown_pose = cur_step.pose;

        // TODO: THIS IS A HACK
        swing_touchdown_pose.translation()[2] += p_swing_foot_touchdown_z_offset_;

        GenerateSwingTraj(swing_foot, swing_foot0, swing_touchdown_pose,
            p_step_height_, plan_time, p_ss_duration_ / 3., p_ss_duration_ / 3., p_ss_duration_ / 3.);

        tracked_bodies_.insert(swing_foot);
        SwitchContact(plan_time);
        DRAKE_DEMAND(tracked_bodies_.size() == 3);

        // change contact
        //SwitchContactState(cur_time);
        cur_state_ = SWING;
        std::cout << "Weight transfer -> Swing @ " << plan_time << std::endl;

        return true;
      }

      break;

    case SWING:
      DRAKE_DEMAND(planned_cs.size() == 1);

      // hack the swing traj if we are past the planned touchdown time
      /*
      if (plan_time >= planned_contact_switch_time) {
        late_touchdown = true;

        BodyMotionData& swing_BMD = get_swing_foot_body_motion_data();
        int last_idx = swing_BMD.trajectory.getNumberOfSegments() - 1;
        Eigen::Matrix<Polynomial<double>, Eigen::Dynamic, Eigen::Dynamic> last_knots = swing_BMD.trajectory.getPolynomialMatrix(last_idx);

        // hack the z position and velocity only
        double t0 = swing_BMD.trajectory.getStartTime(last_idx);
        double t1 = swing_BMD.trajectory.getEndTime(last_idx);
        double z0 = swing_BMD.trajectory.value(t0)(2,0);
        double z1 = swing_BMD.trajectory.value(t1)(2,0) + p_extend_swing_foot_down_z_vel_ * dt;
        double v1 = p_extend_swing_foot_down_z_vel_;
        //Eigen::Vector4d new_z_coeffs = GetCubicSplineCoeffs(t1-t0, z0, z1, v0, v1);
        Eigen::Vector4d new_z_coeffs(z0 + (z1 - z0 - v1 * (t1 -t0)), v1, 0, 0);

        // first one is position
        Polynomial<double> new_z(new_z_coeffs);
        last_knots(2,0) = new_z;
        swing_BMD.trajectory.setPolynomialMatrixBlock(last_knots, last_idx);
        swing_BMD.trajectory.setEndTime(100000.);
      }

      // check for touch down only after half swing
      // if we are in contact switch to the double support contact state and
      // plan/re-plan all trajectories (i.e. zmp, body motion, foot swing etc.)
      if (plan_time >= planned_contact_switch_time - 0.5 * p_ss_duration_ &&
          est_cs.is_foot_in_contact(swing_foot)) {
        // change contact
        SwitchContactState(cur_time);
        cur_state_ = WEIGHT_TRANSFER;
        std::cout << "Swing -> Weight transfer @ " << plan_time << std::endl;

        // dequeue foot steps
        footstep_plan_.pop_front(); // this could potentially be empty after this pop if it's the last step
        step_count_++;
        // generate new trajectories
        GenerateTrajs(plan_time, est_rs.q, est_rs.qd, planned_cs);

        // all tapes assumes 0 sec start, so need to reset clock
        interp_t0_ = cur_time;
        plan_time = cur_time - interp_t0_;
        late_touchdown = false;
      }
      */

      if (plan_time >= planned_contact_switch_time_) {
        cur_state_ = WEIGHT_TRANSFER;
        std::cout << "Swing -> Weight transfer @ " << plan_time << std::endl;

        // dequeue foot steps
        footstep_plan_.pop_front(); // this could potentially be empty after this pop if it's the last step
        step_count_++;
        // generate new trajectories
        GenerateTrajs(plan_time, rs.position(), rs.velocity(), planned_cs);
        tracked_bodies_.erase(tracked_bodies_.find(swing_foot));

        // all tapes assumes 0 sec start, so need to reset clock
        interp_t0_ = cur_time;
        plan_time = cur_time - interp_t0_;
        SwitchContact(plan_time);
        //late_touchdown = false;

        return true;
      }

      break;
  }

  return false;
}

void HumanoidWalkingPlan::GenerateTrajs(double plan_time, const Eigen::VectorXd& est_q, const Eigen::VectorXd& est_qd, const ContactState& planned_cs) {
  KinematicsCache<double> cache_est = robot_->doKinematics(est_q, est_qd);

  // CoM state
  Eigen::Vector4d xcom0;
  xcom0 << robot_->centerOfMass(cache_est).head<2>(),
          (robot_->centerOfMassJacobian(cache_est) * est_qd).head<2>();

  Eigen::Isometry3d feet_pose[2];
  feet_pose[Side::LEFT] = robot_->relativeTransform(cache_est, 0, left_foot_->get_body_index());
  feet_pose[Side::RIGHT] = robot_->relativeTransform(cache_est, 0, right_foot_->get_body_index());

  //////////////////////////////////////////////////////////////////
  // Make ZMP tape.
  Side nxt_stance_foot;
  Side nxt_swing_foot;
  Eigen::Isometry3d nxt_stance_foot_pose;
  Eigen::Isometry3d nxt_swing_foot_pose;
  ContactState nxt_contact_state;
  // Empty queue, going to double support at the center
  if (footstep_plan_.empty()) {
    // these aren't meaningful anymore
    nxt_stance_foot = Side::LEFT;
    nxt_swing_foot = Side::RIGHT;
    nxt_contact_state = double_support();

    nxt_swing_foot_pose = feet_pose[nxt_swing_foot.underlying()];
  } else {
    // single support right
    if (footstep_plan_.front().is_right_foot) {
      nxt_stance_foot = Side::LEFT;
      nxt_swing_foot = Side::RIGHT;
      nxt_contact_state = single_support_left();
    }
    else {
      nxt_stance_foot = Side::RIGHT;
      nxt_swing_foot = Side::LEFT;
      nxt_contact_state = single_support_right();
    }
    nxt_swing_foot_pose = footstep_plan_.front().pose;
  }
  nxt_stance_foot_pose = feet_pose[nxt_stance_foot.underlying()];

  bool is_first_step = is_both_feet_contact(planned_cs);
  double wait_period_before_weight_shift = 0;
  if (is_first_step)
    wait_period_before_weight_shift = 0.5;

  // make zmp
  // this determines how many steps we should be looking ahead
  int num_look_ahead = 3;
  std::vector<Vector2<double>> zmp_d;

  // the foot that we want to shift weight to
  if (footstep_plan_.empty()) {
    // note there is special logic inside planZMP to detect if it's the final footstep or not
    // figure out which foot we just put down
    if (is_foot_contact(planned_cs, Side::LEFT))
      zmp_d.push_back(Footstep2DesiredZMP(Side::RIGHT, feet_pose[Side::RIGHT]));
    else if (is_foot_contact(planned_cs, Side::RIGHT))
      zmp_d.push_back(Footstep2DesiredZMP(Side::LEFT, feet_pose[Side::LEFT]));
    else
      throw std::runtime_error("robot flying.");
  }
  else {
    zmp_d.push_back(Footstep2DesiredZMP(nxt_stance_foot, feet_pose[nxt_stance_foot.underlying()]));
  }

  for (const footstep_t &fs : footstep_plan_) {
    Side side = fs.is_right_foot ? Side::RIGHT : Side::LEFT;
    zmp_d.push_back(Footstep2DesiredZMP(side, fs.pose));
  }

  Eigen::Vector2d zmp_d0, zmpd_d0;
  // figure out where we want the desired zmp for NOW
  // current contact state is double support
  if (is_both_feet_contact(planned_cs)) {
    zmp_d0 = (Footstep2DesiredZMP(Side::LEFT, feet_pose[Side::LEFT]) + Footstep2DesiredZMP(Side::RIGHT, feet_pose[Side::RIGHT])) / 2.;
    zmpd_d0.setZero();
  }
  // single support left
  else if (is_foot_contact(planned_cs, Side::LEFT)) {
    zmp_d0 = Footstep2DesiredZMP(Side::LEFT, feet_pose[Side::LEFT]);
    zmpd_d0.setZero();
    //zmp_d0 = zmp_planner_.GetDesiredZMP(plan_time);
    //zmpd_d0 = zmp_planner_.GetDesiredZMPd(plan_time);
  }
  // single support right
  else if (is_foot_contact(planned_cs, Side::RIGHT)) {
    zmp_d0 = Footstep2DesiredZMP(Side::RIGHT, feet_pose[Side::RIGHT]);
    zmpd_d0.setZero();
    //zmp_d0 = zmp_planner_.GetDesiredZMP(plan_time);
    //zmpd_d0 = zmp_planner_.GetDesiredZMPd(plan_time);
  }
  else {
    throw std::runtime_error("robot flying.");
  }

  PiecewisePolynomial<double> zmp_traj = PlanZMPTraj(zmp_d, num_look_ahead, zmp_d0, zmpd_d0, wait_period_before_weight_shift);
  zmp_planner_.Plan(zmp_traj, xcom0, com_height_);

  //////////////////////////////////////////////////////////////////
  double liftoff_time = wait_period_before_weight_shift + p_ds_duration_;
  double touchdown_time = liftoff_time + p_ss_duration_;
  //double next_liftoff_time = wait_period_before_weight_shift + p_ss_duration_ + p_ds_duration_;

  // q traj
  std::vector<double> time = {0, touchdown_time};
  PiecewisePolynomial<double> q_traj = PiecewisePolynomial<double>::ZeroOrderHold(time, std::vector<MatrixX<double>>(time.size(), est_q));
  dof_tracker_ = VectorTrajectoryTracker<double>(q_traj, Kp_joints_, Kd_joints_);

  // Pelvis traj
  GeneratePelvisTraj(cache_est, p_pelvis_height_, liftoff_time, touchdown_time,
      nxt_stance_foot_pose, nxt_swing_foot_pose);

  // Torso traj
  GenerateTorsoTraj(cache_est, touchdown_time, nxt_swing_foot_pose);

  // Contact
  if (footstep_plan_.empty()) {
    time = {0, INFINITY};
    std::vector<ContactState> contacts(1);
    contacts[0] = double_support();
    contacts_traj_ = PiecewiseContactInformation(time, contacts);
  } else {
    time = {0, liftoff_time, touchdown_time};
    std::vector<ContactState> contacts(2);
    contacts[0] = double_support();
    contacts[1] = nxt_contact_state;
    contacts_traj_ = PiecewiseContactInformation(time, contacts);
  }
}

PiecewisePolynomial<double> HumanoidWalkingPlan::PlanZMPTraj(const std::vector<Eigen::Vector2d> &zmp_d, int num_of_zmp_knots, const Eigen::Vector2d &zmp_d0, const Eigen::Vector2d &zmpd_d0, double time_before_first_weight_shift) const {
  if (zmp_d.size() < 1)
    throw std::runtime_error("zmp_d traj must have size >= 1");

  std::vector<double> zmp_T; // double support weight transfer, followed by single support
  std::vector<Eigen::MatrixXd> zmp_knots;

  int min_size = std::min(num_of_zmp_knots, (int)zmp_d.size());

  double cur_time = 0;
  zmp_knots.push_back(zmp_d0);
  zmp_T.push_back(cur_time);

  if (time_before_first_weight_shift > 0) {
    cur_time += time_before_first_weight_shift;
    // hold current place for sometime for the first step
    zmp_knots.push_back(zmp_d0);
    zmp_T.push_back(cur_time);
  }

  for (int i = 0; i < min_size; i++) {
    // add a double support phase
    cur_time += p_ds_duration_;
    // last step, need to stop in the middle
    if (i == min_size - 1) {
      zmp_knots.push_back((zmp_knots.back() + zmp_d[i]) / 2.);
      zmp_T.push_back(cur_time);
    }
    else {
      zmp_knots.push_back(zmp_d[i]);
      zmp_T.push_back(cur_time);
    }

    // add a single support phase
    cur_time += p_ss_duration_;
    zmp_knots.push_back(zmp_knots.back());
    zmp_T.push_back(cur_time);
  }

  return PiecewisePolynomial<double>::Pchip(zmp_T, zmp_knots);
}

Eigen::Vector2d HumanoidWalkingPlan::Footstep2DesiredZMP(Side side, const Eigen::Isometry3d &step) const {
  double zmp_shift_in = 0;
  const ContactInformation* foot_contact = &all_contactable_bodies_.at(left_foot_);
  // zmp_shift_in = p_left_foot_zmp_y_shift_;
  if (side == Side::RIGHT) {
    foot_contact = &all_contactable_bodies_.at(right_foot_);
    // zmp_shift_in = p_right_foot_zmp_y_shift_;
  }
  const Eigen::Matrix3Xd &offsets = foot_contact->contact_points();
  Eigen::Vector3d avg_contact_offset(Eigen::Vector3d::Zero());
  for (int i = 0; i < offsets.cols(); i++)
    avg_contact_offset += offsets.col(i);
  avg_contact_offset = avg_contact_offset / offsets.cols();

  // offset y on in body frame
  avg_contact_offset[1] += zmp_shift_in;
  Eigen::Isometry3d mid_stance_foot(Eigen::Isometry3d::Identity());
  mid_stance_foot.translation() = avg_contact_offset;
  mid_stance_foot = step * mid_stance_foot;
  return mid_stance_foot.translation().head(2);
}

void HumanoidWalkingPlan::GeneratePelvisTraj(const KinematicsCache<double>& cache,
    double pelvis_height_above_sole,
    double liftoff_time, double next_liftoff_time,
    const Eigen::Isometry3d& nxt_stance_foot_pose,
    const Eigen::Isometry3d& nxt_swing_foot_pose) {

  Isometry3<double> pose = robot_->relativeTransform(cache, 0, pelvis_->get_body_index());
  std::vector<double> time = {0, liftoff_time, next_liftoff_time};
  std::vector<MatrixX<double>> pos_d(time.size(), pose.translation());
  eigen_aligned_std_vector<Quaternion<double>> rot_d(time.size(), Quaternion<double>(pose.linear()));

  // Position XY will be ignored.
  pos_d[1](2, 0) = nxt_stance_foot_pose.translation()[2] + pelvis_height_above_sole;
  double yaw1 = get_yaw(nxt_stance_foot_pose.linear());
  rot_d[1] = Quaternion<double>(AngleAxis<double>(yaw1, Vector3<double>::UnitZ()));

  pos_d[2](2, 0) = nxt_swing_foot_pose.translation()[2] + pelvis_height_above_sole;
  double yaw2 = get_yaw(nxt_swing_foot_pose.linear());
  rot_d[2] = Quaternion<double>(AngleAxis<double>(yaw2, Vector3<double>::UnitZ()));

  PiecewisePolynomial<double> pos_traj = PiecewisePolynomial<double>::FirstOrderHold(time, pos_d);
  PiecewiseQuaternionSlerp<double> rot_traj = PiecewiseQuaternionSlerp<double>(time, rot_d);

  body_trackers_[pelvis_] = CartesianTrajectoryTracker<double>(pos_traj, rot_traj, Kp_pelvis_, Kd_pelvis_);
}

void HumanoidWalkingPlan::GenerateSwingTraj(const RigidBody<double>* swing_foot, const Isometry3<double>& foot0, const Isometry3<double>& foot1, double mid_z_offset, double pre_swing_dur, double swing_up_dur, double swing_transfer_dur, double swing_down_dur) {
  double z_height_mid_swing = std::max(foot1.translation()[2], foot0.translation()[2]) + mid_z_offset;
  int num_T = 5;
  std::vector<double> time(num_T);
  time[0] = 0;
  time[1] = pre_swing_dur;
  time[2] = pre_swing_dur + swing_up_dur;
  time[3] = pre_swing_dur + swing_up_dur + swing_transfer_dur;
  time[4] = pre_swing_dur + swing_up_dur + swing_transfer_dur + swing_down_dur;

  std::vector<MatrixX<double>> pos_d(num_T, foot0.translation());
  std::vector<MatrixX<double>> vel_d(num_T, VectorX<double>::Zero(3));
  eigen_aligned_std_vector<Quaternion<double>> rot_d(num_T, Quaternion<double>(foot0.linear()));

  // xy will be set later
  pos_d[2] = foot1.translation();
  rot_d[2] = foot1.linear();
  pos_d[2](2, 0) = z_height_mid_swing;
  // right above the touchdown pose, only increase z
  pos_d[3] = foot1.translation();
  rot_d[3] = foot1.linear();
  pos_d[3](2, 0) = z_height_mid_swing;

  pos_d[4] = foot1.translation();
  rot_d[4] = foot1.linear();

  // helper method to figure out x,y velocity at midpoint
  std::vector<double> T_xy = {0, swing_up_dur + swing_transfer_dur};
  std::vector<MatrixX<double>> xy_pos = {foot0.translation().head<2>(), foot1.translation().head<2>()};
//  std::vector<MatrixX<double>> xy_vel(2, Eigen::Vector2d::Zero());

  // Hmmm this seems wrong
  //auto xy_spline = PiecewisePolynomial<double>::Cubic(T_xy, xy_pos, xy_vel);
  auto xy_spline = PiecewisePolynomial<double>::FirstOrderHold(T_xy, xy_pos);
  auto xy_spline_deriv = xy_spline.derivative();

  pos_d[2].col(0).head<2>() = xy_spline.value(swing_up_dur);
  vel_d[2].col(0).head<2>() = xy_spline_deriv.value(swing_up_dur);

  // end velocity specification for spline
  vel_d[4](2, 0) = p_swing_foot_touchdown_z_vel_;

  PiecewisePolynomial<double> pos_traj = PiecewisePolynomial<double>::Cubic(time, pos_d, vel_d);
  PiecewiseQuaternionSlerp<double> rot_traj = PiecewiseQuaternionSlerp<double>(time, rot_d);

  body_trackers_[swing_foot] = CartesianTrajectoryTracker<double>(pos_traj, rot_traj, Kp_foot_, Kd_foot_);
}

void HumanoidWalkingPlan::GenerateTorsoTraj(const KinematicsCache<double>& cache,
    double next_liftoff_time,
    const Eigen::Isometry3d& nxt_swing_foot_pose) {

  Isometry3<double> pose = robot_->relativeTransform(cache, 0, torso_->get_body_index());
  std::vector<double> time = {0, next_liftoff_time};
  std::vector<MatrixX<double>> pos_d(time.size(), pose.translation());
  eigen_aligned_std_vector<Quaternion<double>> rot_d(time.size(), Quaternion<double>(pose.linear()));

  // Position will be ignored.
  double yaw = get_yaw(nxt_swing_foot_pose.linear());
  rot_d[1] = Quaternion<double>(AngleAxis<double>(yaw, Vector3<double>::UnitZ()));

  PiecewisePolynomial<double> pos_traj = PiecewisePolynomial<double>::FirstOrderHold(time, pos_d);
  PiecewiseQuaternionSlerp<double> rot_traj = PiecewiseQuaternionSlerp<double>(time, rot_d);

  body_trackers_[torso_] = CartesianTrajectoryTracker<double>(pos_traj, rot_traj, Kp_pelvis_, Kd_pelvis_);
}

void HumanoidWalkingPlan::SwitchContact(double plan_time) {
  int seg_idx = contacts_traj_.getSegmentIndex(plan_time);
  planned_contact_switch_time_ = contacts_traj_.getEndTime(seg_idx);
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
