#include "drake/examples/kuka_iiwa_arm/jjz_common.h"
#include "drake/multibody/constraint/rigid_body_constraint.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/rigid_body_ik.h"

#include "drake/examples/PlanarPushing/dubins_interface.h"
#include "drake/examples/PlanarPushing/pushing_multi_actions.h"
#include "drake/util/drakeUtil.h"

#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace jjz {

const std::string kEEName("iiwa_link_7");

IiwaState::IiwaState(const RigidBodyTree<double>* iiwa,
                     const RigidBodyFrame<double>* frame_T)
    : iiwa_(iiwa),
      frame_T_(frame_T),
      cache_(iiwa_->CreateKinematicsCache()),
      q_(VectorX<double>::Zero(iiwa_->get_num_positions())),
      v_(VectorX<double>::Zero(iiwa_->get_num_velocities())),
      trq_(VectorX<double>::Zero(iiwa_->get_num_actuators())),
      ext_trq_(VectorX<double>::Zero(iiwa_->get_num_actuators())) {
  DRAKE_DEMAND(iiwa_->get_num_positions() == iiwa_->get_num_velocities());
  DRAKE_DEMAND(iiwa_->get_num_actuators() == iiwa_->get_num_velocities());
}

bool IiwaState::UpdateState(const lcmt_iiwa_status& msg) {
  // Check msg.
  if (msg.num_joints != iiwa_->get_num_positions()) {
    std::cout << "msg joints: " << msg.num_joints << std::endl;
  }
  DRAKE_DEMAND(msg.num_joints == iiwa_->get_num_positions());

  const double cur_time = msg.utime / 1e6;
  // Same time stamp, should just return.
  if (init_ && cur_time == time_) return false;

  if (init_) {
    delta_time_ = cur_time - time_;
  } else {
    delta_time_ = 0;
  }

  // Update time, position, and torque.
  time_ = cur_time;
  for (int i = 0; i < msg.num_joints; ++i) {
    q_[i] = msg.joint_position_measured[i];
    v_[i] = msg.joint_velocity_estimated[i];
    trq_[i] = msg.joint_torque_measured[i];
    ext_trq_[i] = msg.joint_torque_external[i];
  }

  // Update kinematics.
  cache_.initialize(q_, v_);
  iiwa_->doKinematics(cache_);
  J_ = iiwa_->CalcFrameSpatialVelocityJacobianInWorldFrame(cache_, *frame_T_);

  // ext_trq = trq_measured - trq_id
  //         = M * qdd + h - J^T * F - (M * qdd + h)
  //         = -J^T * F
  // I think the measured trq_ext is the flip side.
  ext_wrench_ = J_.transpose().colPivHouseholderQr().solve(ext_trq_);
  // ext_wrench_ = J_.transpose().colPivHouseholderQr().solve(-ext_trq_);

  X_WT_ = iiwa_->CalcFramePoseInWorldFrame(cache_, *frame_T_);
  V_WT_ = iiwa_->CalcFrameSpatialVelocityInWorldFrame(cache_, *frame_T_);

  init_ = true;

  return true;
}

void FillDebugMessage(const IiwaState& state, lcmt_jjz_controller* msg) {
  msg->utime = state.get_time() * 1e6;
  msg->dt = state.get_dt();

  eigenVectorToCArray(state.get_q(), msg->q);
  eigenVectorToCArray(state.get_v(), msg->v);

  Eigen::Matrix<double, 7, 1> tmp_pose = jjz::pose_to_vec(state.get_X_WT());
  eigenVectorToCArray(tmp_pose, msg->X_WT);
  eigenVectorToCArray(state.get_V_WT(), msg->V_WT);

  eigenVectorToCArray(state.get_ext_wrench(), msg->ext_wrench);
}

Eigen::Matrix<double, 7, 1> pose_to_vec(const Isometry3<double>& pose) {
  Eigen::Matrix<double, 7, 1> ret;
  ret.head<3>() = pose.translation();

  Quaternion<double> quat(pose.linear());
  ret[3] = quat.w();
  ret[4] = quat.x();
  ret[5] = quat.y();
  ret[6] = quat.z();

  return ret;
}

VectorX<double> PointIk(const Isometry3<double>& X_WT,
                        const RigidBodyFrame<double>& frame_T,
                        const VectorX<double>& q_ini,
                        RigidBodyTree<double>* robot) {
  std::cout << "PointIk: X_WT:\n" << X_WT.matrix() << "\n\n";

  std::vector<RigidBodyConstraint*> constraint_array;

  IKoptions ikoptions(robot);

  Isometry3<double> X_WE = X_WT * frame_T.get_transform_to_body().inverse();

  Vector3<double> pos_tol(0.001, 0.001, 0.001);
  double rot_tol = 0.001;
  Vector3<double> pos_lb = X_WE.translation() - pos_tol;
  Vector3<double> pos_ub = X_WE.translation() + pos_tol;

  WorldPositionConstraint pos_con(
      robot, frame_T.get_rigid_body().get_body_index(), Vector3<double>::Zero(),
      pos_lb, pos_ub, Vector2<double>::Zero());

  constraint_array.push_back(&pos_con);

  // Adds a rotation constraint.
  WorldQuatConstraint quat_con(robot, frame_T.get_rigid_body().get_body_index(),
                               math::rotmat2quat(X_WE.linear()), rot_tol,
                               Vector2<double>::Zero());
  constraint_array.push_back(&quat_con);

  VectorX<double> q_res = VectorX<double>::Zero(7);
  VectorX<double> zero = VectorX<double>::Zero(7);

  int info;
  std::vector<std::string> infeasible_constraints;
  inverseKin(robot, q_ini, zero, constraint_array.size(),
             constraint_array.data(), ikoptions, &q_res, &info,
             &infeasible_constraints);

  DRAKE_DEMAND(info == 1);
  return q_res;
}

// Returned stuff is in ZZJ's Goal frame. 0 is origin of Goal frame.
manipulation::PiecewiseCartesianTrajectory<double>
PlanPlanarPushingTrajMultiAction(const Vector3<double>& x_GQ,
                                 const Isometry3<double>& X_WG, double duration,
                                 std::string load_file_name) {
  std::unique_ptr<MultiPushActionsPlanner> multi_action_planner;

  if (!load_file_name.empty()) {
    std::cout << "about to load graph" << std::endl;
    multi_action_planner =
        std::make_unique<MultiPushActionsPlanner>(load_file_name);
  } else {
    double height = 0.234;
    double width = 0.178;
    double rho = width / 5;

    // Limit surface A_11. If you are unsure, just set it to be 1.
    double ls_a = 1;
    // Limit surface A_33 / rho^2, where rho is a characteristic length,
    // similar
    // to the minimum bounding circle radius.
    double ls_b = ls_a / (rho * rho);
    // Coefficient of contact friction
    double mu = 0.4;

    int num_actions = 4;
    Eigen::Matrix<double, Eigen::Dynamic, 2> ct_pts(num_actions, 2);
    Eigen::Matrix<double, Eigen::Dynamic, 2> normal_pts(num_actions, 2);
    ct_pts << 0, -height / 2, -width / 2, 0, 0, height / 2, width / 2, 0;
    normal_pts << 0, 1, 1, 0, 0, -1, -1, 0;

    std::vector<Eigen::Vector2d> all_contact_points;
    std::vector<Eigen::Vector2d> all_normals;
    for (int i = 0; i < ct_pts.rows(); ++i) {
      all_contact_points.push_back(ct_pts.row(i).transpose());
      all_normals.push_back(normal_pts.row(i).transpose());
    }
    multi_action_planner = std::make_unique<MultiPushActionsPlanner>(
        all_contact_points, all_normals, mu, ls_a, ls_b);
    double xmin, xmax, ymin, ymax;
    xmin = -0.01;
    xmax = 0.3;
    ymin = -0.1;
    ymax = 0.1;
    // xmin = -0.2; xmax = 0.2; ymin = -0.2; ymax = 0.2;
    multi_action_planner->SetWorkSpaceBoxConstraint(xmin, xmax, ymin, ymax);

    int num_samples_se2 = 300;
    double switching_action_cost = 0.05;
    multi_action_planner->SetGraphSize(num_samples_se2);

    // goal set.
    double goal_x_range = 0.05;
    double goal_y_range = 0.05;
    double goal_theta_range = M_PI / 4.0;
    int num_goals = 100;
    multi_action_planner->SetGoalSet(goal_x_range, goal_y_range,
                                     goal_theta_range, num_goals);

    multi_action_planner->SetActionSwitchCost(switching_action_cost);
    multi_action_planner->ConstructPlanningGraph();
    std::string output_file_name = "graph.txt";
    std::ofstream output_ss(output_file_name);
    multi_action_planner->Serialize(output_ss);
    output_ss.close();

    std::cout << "graph generated. press Enter to continue.\n";
    getchar();
  }  // Complete construction of graph planner.

  std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> all_object_poses;
  std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> all_pusher_poses;
  std::vector<int> action_id;

  int num_way_pts_perseg = 100;
  ////////////////////////////////////////////

  multi_action_planner->Plan(x_GQ, num_way_pts_perseg, &action_id,
                             &all_object_poses, &all_pusher_poses);

  for (unsigned i = 0; i < all_object_poses.size(); ++i) {
    std::cout << "Segment Traj " << i << std::endl;
    std::cout << "Action id " << action_id[i] << std::endl;
    for (unsigned j = 0; j < all_object_poses[i].rows(); ++j) {
      std::cout << all_object_poses[i].row(j) << std::endl;
    }
  }

  int num_switches = 0;
  for (unsigned i = 0; i < action_id.size() - 1; ++i) {
    if (action_id[i] != action_id[i + 1]) {
      num_switches++;
    }
  }

  int num_action_segs = all_object_poses.size();
  double dt = duration / (num_way_pts_perseg - 1);
  double time_lift_up = 2.0;
  double time_move_above = 2.0;
  double time_move_down = 2.0;
  double dist_lift_up = 0.1;

  int num_points_per_seg = all_pusher_poses[0].rows();
  int total_way_points =
      num_points_per_seg * num_action_segs + 3 * (num_switches);
  std::vector<double> times(total_way_points);
  std::vector<Isometry3<double>> X_WT(total_way_points,
                                      Isometry3<double>::Identity());

  /*
  std::vector<MatrixX<double>> pos(total_way_points,
                                   MatrixX<double>::Zero(3, 1));
  eigen_aligned_std_vector<Quaternion<double>> rot(total_way_points);
  */

  double cur_time = 0.0;
  int index = 0;
  for (int id_traj = 0; id_traj < num_action_segs; ++id_traj) {
    Eigen::Matrix<double, Eigen::Dynamic, 3> pusher_poses =
        all_pusher_poses[id_traj];
    int num_way_points = pusher_poses.rows();
    for (int i = 0; i < num_way_points; ++i) {
      cur_time = cur_time + dt;
      times[index] = cur_time;

      // This is really X_GT
      X_WT[index].translation()[0] = pusher_poses(i, 0);
      X_WT[index].translation()[1] = pusher_poses(i, 1);

      // pos[index](0, 0) = pusher_poses(i, 0);
      // pos[index](1, 0) = pusher_poses(i, 1);

      // std::cout << index << " : " << cur_time <<"," << pos[index](0, 0) <<
      // "," << pos[index](1, 0) << std::endl;
      // std::cout << "jjz: pusher pose" << pusher_poses.row(i) << "\n";
      // std::cout << "jjz: object pose" << object_poses.row(i) << "\n";

      // sfeng thinks jjz's angle is somehow 90 deg off from me.
      std::cout << "t: " << cur_time << ", pose " << pusher_poses.row(i)
                << "\n";
      Matrix3<double> X_GT(AngleAxis<double>(pusher_poses(i, 2) + M_PI / 2.,
                                             Vector3<double>::UnitZ()));
      // rot[index] = Quaternion<double>(X_GT);
      X_WT[index].linear() = X_GT;
      X_WT[index] = X_WG * X_WT[index];

      ++index;
    }
    if (id_traj < num_action_segs - 1 &&
        action_id[id_traj] != action_id[id_traj + 1]) {
      // The robot first moves up.
      cur_time = cur_time + time_lift_up;
      times[index] = cur_time;
      /*
      pos[index] = pos[index - 1];
      // Add z value.
      pos[index](2, 0) = dist_lift_up;
      rot[index] = rot[index - 1];
      */
      X_WT[index] = X_WT[index - 1];
      X_WT[index].translation()[2] = dist_lift_up;

      index++;
      // The robot then moves to the plane above the next pushing location and
      // align with the initial pose of the next trajectory.
      cur_time = cur_time + time_move_above;
      times[index] = cur_time;
      Eigen::Vector3d nxt_push_pose =
          all_pusher_poses[id_traj + 1].row(0).transpose();
      /*
      pos[index](0, 0) = nxt_push_pose(0);
      pos[index](1, 0) = nxt_push_pose(1);
      pos[index](2, 0) = dist_lift_up;
      */
      // This is really X_GT
      X_WT[index].translation()(0) = nxt_push_pose(0);
      X_WT[index].translation()(1) = nxt_push_pose(1);
      X_WT[index].translation()(2) = dist_lift_up;

      std::cout << "t: " << cur_time << ", pose " << nxt_push_pose.transpose()
                << "\n";
      Matrix3<double> X_GT(AngleAxis<double>(nxt_push_pose(2) + M_PI / 2.,
                                             Vector3<double>::UnitZ()));
      // rot[index] = Quaternion<double>(X_GT);
      X_WT[index].linear() = X_GT;
      X_WT[index] = X_WG * X_WT[index];

      index++;
      // The robot then moves down to the next pushing location.
      cur_time = cur_time + time_move_down;
      times[index] = cur_time;
      /*
      pos[index](0, 0) = nxt_push_pose(0);
      pos[index](1, 0) = nxt_push_pose(1);
      rot[index] = rot[index - 1];
      */
      X_WT[index].translation()[0] = nxt_push_pose(0);
      X_WT[index].translation()[1] = nxt_push_pose(1);
      X_WT[index].linear() = X_GT;
      X_WT[index] = X_WG * X_WT[index];
      index++;
    }
  }
  DRAKE_DEMAND(index == total_way_points);

  std::cout << "i tot" << index << " " << total_way_points << "\n";

  std::vector<MatrixX<double>> pos(total_way_points,
                                   MatrixX<double>::Zero(3, 1));
  eigen_aligned_std_vector<Quaternion<double>> rot(total_way_points);
  for (int i = 0; i < total_way_points; i++) {
    pos[i] = X_WT[i].translation();
    rot[i] = Quaternion<double>(X_WT[i].linear());
  }

  PiecewiseQuaternionSlerp<double> rot_traj(times, rot);
  PiecewisePolynomial<double> pos_traj =
      PiecewisePolynomial<double>::FirstOrderHold(times, pos);
  manipulation::PiecewiseCartesianTrajectory<double> traj(pos_traj, rot_traj);

  return traj;
}

static Matrix3<double> FlatYAxisFrame(const Vector3<double>& z) {
  const Vector3<double> py_z = z.normalized();
  const Vector3<double> py_z_proj = Vector3<double>(z(0), z(1), 0).normalized();
  const Vector3<double> py_y =
      py_z_proj.cross(Vector3<double>::UnitZ()).normalized();
  const Vector3<double> py_x = py_y.cross(py_z).normalized();
  Matrix3<double> rot;
  rot.col(0) = py_x;
  rot.col(1) = py_y;
  rot.col(2) = py_z;
  return rot;
}

VectorX<double> GazeIk(const Vector3<double>& target_in_world,
                       const Vector3<double>& camera_in_world,
                       const RigidBodyFrame<double>& frame_C,
                       const VectorX<double>& q_ini,
                       RigidBodyTree<double>* robot) {
  std::vector<RigidBodyConstraint*> constraint_array;

  IKoptions ikoptions(robot);

  const Isometry3<double>& X_BC = frame_C.get_transform_to_body();
  const int body_idx = frame_C.get_rigid_body().get_body_index();

  // Gaze dir constraint.
  const Vector3<double> gaze_ray_dir_in_body = X_BC.linear().col(2);
  const Vector3<double> gaze_ray_origin_in_body = X_BC.translation();

  WorldGazeTargetConstraint con(robot, body_idx, gaze_ray_dir_in_body,
                                target_in_world, gaze_ray_origin_in_body, 0.01,
                                Vector2<double>::Zero());

  constraint_array.push_back(&con);

  // Camera position constraint.
  Vector3<double> p_WB =
      (Eigen::Translation<double, 3>(camera_in_world) * X_BC.inverse())
          .translation();
  Vector3<double> pos_tol(0.001, 0.001, 0.001);
  Vector3<double> pos_lb = p_WB - pos_tol;
  Vector3<double> pos_ub = p_WB + pos_tol;

  WorldPositionConstraint pos_con(robot, body_idx, Vector3<double>::Zero(),
                                  pos_lb, pos_ub, Vector2<double>::Zero());

  constraint_array.push_back(&pos_con);

  VectorX<double> q_res = VectorX<double>::Zero(7);
  VectorX<double> zero = VectorX<double>::Zero(7);

  int info;
  std::vector<std::string> infeasible_constraints;
  inverseKin(robot, q_ini, zero, constraint_array.size(),
             constraint_array.data(), ikoptions, &q_res, &info,
             &infeasible_constraints);

  DRAKE_DEMAND(info == 1);
  return q_res;
}

VectorX<double> GazeIk2(const Vector3<double>& target_in_world,
                        const Vector3<double>& target_to_camera_in_world,
                        double min_dist, const RigidBodyFrame<double>& frame_C,
                        const VectorX<double>& q_ini,
                        RigidBodyTree<double>* robot) {
  std::vector<RigidBodyConstraint*> constraint_array;

  IKoptions ikoptions(robot);

  const Isometry3<double>& X_BC = frame_C.get_transform_to_body();
  const int body_idx = frame_C.get_rigid_body().get_body_index();

  // Gaze dir constraint.
  const Vector3<double> gaze_ray_dir_in_body = X_BC.linear().col(2);
  const Vector3<double> gaze_ray_origin_in_body = X_BC.translation();

  WorldGazeTargetConstraint con(robot, body_idx, gaze_ray_dir_in_body,
                                target_in_world, gaze_ray_origin_in_body, 0.01,
                                Vector2<double>::Zero());

  constraint_array.push_back(&con);

  // Camera position constraint.
  Isometry3<double> X_WTgt = Isometry3<double>::Identity();
  X_WTgt.translation() = target_in_world;
  X_WTgt.linear() = FlatYAxisFrame(target_to_camera_in_world);

  Vector3<double> pos_lb(0, 0, min_dist);
  Vector3<double> pos_ub(0, 0, std::numeric_limits<double>::infinity());

  WorldPositionInFrameConstraint pos_con(robot, body_idx, X_BC.translation(),
                                         X_WTgt.matrix(), pos_lb, pos_ub,
                                         Vector2<double>::Zero());

  constraint_array.push_back(&pos_con);

  VectorX<double> q_res = VectorX<double>::Zero(7);
  VectorX<double> zero = VectorX<double>::Zero(7);

  int info;
  std::vector<std::string> infeasible_constraints;
  inverseKin(robot, q_ini, zero, constraint_array.size(),
             constraint_array.data(), ikoptions, &q_res, &info,
             &infeasible_constraints);

  DRAKE_DEMAND(info == 1);
  return q_res;
}

std::vector<VectorX<double>> ComputeCalibrationConfigurations(
    const RigidBodyTree<double>& robot, const RigidBodyFrame<double>& frame_C,
    const VectorX<double>& q0, const Vector3<double>& p_WP, double min_dist,
    double width, double height, int num_width_pt, int num_height_pt) {
  KinematicsCache<double> cache = robot.CreateKinematicsCache();
  cache.initialize(q0);
  robot.doKinematics(cache);

  const Isometry3<double> X_WC0 =
      robot.CalcFramePoseInWorldFrame(cache, frame_C);
  const Vector3<double> C0_to_P = p_WP - X_WC0.translation();
  const double pyramid_height = C0_to_P.norm();

  Isometry3<double> X_WP = Isometry3<double>::Identity();
  X_WP.linear() = FlatYAxisFrame(C0_to_P);
  X_WP.translation() = p_WP;

  double dw = width / (num_width_pt - 1);
  double dh = height / (num_height_pt - 1);

  std::vector<VectorX<double>> ret;
  for (int i = 0; i < num_width_pt; i++) {
    for (int j = 0; j < num_height_pt; j++) {
      Vector3<double> p_PC(-height / 2. + j * dh, -width / 2. + i * dw,
                           -pyramid_height);
      p_PC = p_PC.normalized();  // * pyramid_height;
      Vector3<double> p_WC = X_WP * p_PC;
      // ret.push_back(GazeIk(p_WP, p_WC, frame_C, q0,
      // (RigidBodyTree<double>*)&robot));
      ret.push_back(GazeIk2(p_WP, p_WC - p_WP, min_dist, frame_C, q0,
                            (RigidBodyTree<double>*)&robot));
    }
  }

  return ret;
}

static std::vector<std::vector<std::vector<solvers::VectorDecisionVariable<1>>>>
resize_var(int num_seg, int rows, int cols) {
  std::vector<std::vector<std::vector<solvers::VectorDecisionVariable<1>>>> ret(
      num_seg);
  for (int t = 0; t < num_seg; t++) {
    ret[t].resize(rows);
    for (int r = 0; r < rows; r++) {
      ret[t][r].resize(cols);
    }
  }
  return ret;
}

// Assuming 3rd order
static bool CheckVelocityAndAccConstraints3(const Polynomial<double>& poly,
                                            double time, double v_lower,
                                            double v_upper, double vd_lower,
                                            double vd_upper) {
  Polynomial<double> v = poly.Derivative();
  Polynomial<double> vd = v.Derivative();

  DRAKE_DEMAND(poly.GetDegree() == 3);

  // Check vd.
  if (vd.EvaluateUnivariate(0) < vd_lower ||
      vd.EvaluateUnivariate(0) > vd_upper ||
      vd.EvaluateUnivariate(time) < vd_lower ||
      vd.EvaluateUnivariate(time) > vd_upper) {
    return false;
  }

  // Check v.
  if (v.EvaluateUnivariate(0) < v_lower || v.EvaluateUnivariate(0) > v_upper ||
      v.EvaluateUnivariate(time) < v_lower ||
      v.EvaluateUnivariate(time) > v_upper) {
    return false;
  }
  VectorX<double> coeffs = poly.GetCoefficients();
  double extrema_t = -coeffs(2) / (3. * coeffs(3));
  if (extrema_t > 0 && extrema_t < time) {
    if (v.EvaluateUnivariate(extrema_t) < v_lower ||
        v.EvaluateUnivariate(extrema_t) > v_upper) {
      return false;
    }
  }

  return true;
}

static bool CheckTrajVelAndAccConstraints(
    const PiecewisePolynomial<double>& traj, const MatrixX<double>& v_lower,
    const MatrixX<double>& v_upper, const MatrixX<double>& vd_lower,
    const MatrixX<double>& vd_upper) {
  int num_rows = v_lower.rows();
  int num_cols = v_lower.cols();
  for (int t = 0; t < traj.getNumberOfSegments(); t++) {
    for (int r = 0; r < num_rows; r++) {
      for (int c = 0; c < num_cols; c++) {
        if (!CheckVelocityAndAccConstraints3(
                traj.getPolynomial(t, r, c), traj.getDuration(t), v_lower(r, c),
                v_upper(r, c), vd_lower(r, c), vd_upper(r, c))) {
          return false;
        }
      }
    }
  }

  return true;
}

static PiecewisePolynomial<double> GuessTrajTime3(
    const std::vector<MatrixX<double>>& q, const MatrixX<double>& v0,
    const MatrixX<double>& v1, const MatrixX<double>& v_lower,
    const MatrixX<double>& v_upper, const MatrixX<double>& vd_lower,
    const MatrixX<double>& vd_upper) {
  std::vector<double> times(q.size(), 0);
  std::vector<double> dt(q.size() - 1, 1);

  for (size_t t = 1; t < q.size(); t++) {
    times[t] = times[t - 1] + dt[t - 1];
  }

  PiecewisePolynomial<double> traj =
      PiecewisePolynomial<double>::Cubic(times, q, v0, v1);

  // Check constraints, and adjust dt.
  int num_rows = v0.rows();
  int num_cols = v0.cols();

  while (true) {
    bool all_ok = true;
    for (size_t t = 0; t < q.size() - 1; t++) {
      bool ok = true;
      for (int r = 0; r < num_rows; r++) {
        for (int c = 0; c < num_cols; c++) {
          ok &= CheckVelocityAndAccConstraints3(
              traj.getPolynomial(t, r, c), dt[t], v_lower(r, c), v_upper(r, c),
              vd_lower(r, c), vd_upper(r, c));
        }
      }
      if (!ok) {
        dt[t] = 1.5 * dt[t];
        all_ok = false;
      }
    }

    if (all_ok) break;

    for (size_t t = 1; t < q.size(); t++) {
      times[t] = times[t - 1] + dt[t - 1];
    }
    traj = PiecewisePolynomial<double>::Cubic(times, q, v0, v1);
  }

  return traj;
}

static PiecewisePolynomial<double> LineSearchSingleCubicSpline(
    const std::vector<MatrixX<double>>& q, double min_time, double max_time,
    const MatrixX<double>& v0, const MatrixX<double>& v1,
    const MatrixX<double>& v_lower, const MatrixX<double>& v_upper,
    const MatrixX<double>& vd_lower, const MatrixX<double>& vd_upper) {
  DRAKE_DEMAND(q.size() == 2);

  double mid_time = (min_time + max_time) / 2.;
  std::vector<double> times = {0, mid_time};
  PiecewisePolynomial<double> traj =
      PiecewisePolynomial<double>::Cubic(times, q, v0, v1);

  if (CheckTrajVelAndAccConstraints(traj, v_lower, v_upper, vd_lower,
                                    vd_upper)) {
    if (std::fabs(mid_time - min_time) < 1e-2) {
      return traj;
    } else {
      return LineSearchSingleCubicSpline(q, min_time, mid_time, v0, v1, v_lower,
                                         v_upper, vd_lower, vd_upper);
    }
  } else {
    return LineSearchSingleCubicSpline(q, mid_time, max_time, v0, v1, v_lower,
                                       v_upper, vd_lower, vd_upper);
  }
}

PiecewisePolynomial<double> LineSearchSingleCubicSpline(
    const std::vector<MatrixX<double>>& q, const MatrixX<double>& v0,
    const MatrixX<double>& v1, const MatrixX<double>& v_lower,
    const MatrixX<double>& v_upper, const MatrixX<double>& vd_lower,
    const MatrixX<double>& vd_upper) {
  DRAKE_DEMAND(q.size() == 2);

  PiecewisePolynomial<double> max_time_traj =
      GuessTrajTime3(q, v0, v1, v_lower, v_upper, vd_lower, vd_upper);

  return LineSearchSingleCubicSpline(q, 0.1, max_time_traj.getEndTime(), v0, v1,
                                     v_lower, v_upper, vd_lower, vd_upper);
}

PiecewisePolynomial<double> RetimeTrajCubic(
    const std::vector<MatrixX<double>>& q, const MatrixX<double>& v0,
    const MatrixX<double>& v1, const MatrixX<double>& v_lower,
    const MatrixX<double>& v_upper, const MatrixX<double>& vd_lower,
    const MatrixX<double>& vd_upper) {
  if (q.size() == 2) {
    return LineSearchSingleCubicSpline(q, v0, v1, v_lower, v_upper, vd_lower,
                                       vd_upper);
  } else {
    return GuessTrajTime3(q, v0, v1, v_lower, v_upper, vd_lower, vd_upper);
  }
}

PiecewisePolynomial<double> RetimeTrajCubicDOESNTWORK(
    const std::vector<MatrixX<double>>& q, const MatrixX<double>& v0,
    const MatrixX<double>& v1, const MatrixX<double>& v_lower,
    const MatrixX<double>& v_upper, const MatrixX<double>& vd_lower,
    const MatrixX<double>& vd_upper) {
  const int num_segments = q.size() - 1;
  const int num_rows = q.front().rows();
  const int num_cols = q.front().cols();

  const PiecewisePolynomial<double> initial_guess =
      GuessTrajTime3(q, v0, v1, v_lower, v_upper, vd_lower, vd_upper);
  VectorX<double> guess_coeffs;

  solvers::MathematicalProgram prog;
  std::vector<solvers::VectorDecisionVariable<1>> t3(num_segments);
  std::vector<solvers::VectorDecisionVariable<1>> t2(num_segments);
  std::vector<solvers::VectorDecisionVariable<1>> t1(num_segments);
  std::vector<std::vector<std::vector<solvers::VectorDecisionVariable<1>>>> c3 =
      resize_var(num_segments, num_rows, num_cols);
  std::vector<std::vector<std::vector<solvers::VectorDecisionVariable<1>>>> c2 =
      c3;
  std::vector<std::vector<std::vector<solvers::VectorDecisionVariable<1>>>> c1 =
      c3;

  for (int t = 0; t < num_segments; t++) {
    const double t_i = initial_guess.getDuration(t);

    t3[t] = prog.NewContinuousVariables<1, 1>(1, 1,
                                              "t3[" + std::to_string(t) + "]");
    t2[t] = prog.NewContinuousVariables<1, 1>(1, 1,
                                              "t2[" + std::to_string(t) + "]");
    t1[t] = prog.NewContinuousVariables<1, 1>(1, 1,
                                              "t1[" + std::to_string(t) + "]");

    prog.SetInitialGuess(t1[t], Vector1<double>(t_i));
    prog.SetInitialGuess(t2[t], Vector1<double>(std::pow(t_i, 2)));
    prog.SetInitialGuess(t3[t], Vector1<double>(std::pow(t_i, 3)));

    std::cout << "** initial guess(" << t
              << "): " << initial_guess.getDuration(t) << "\n";

    // t1[t](0) * t1[t](0) == t2[t](0)
    {
      MatrixX<double> Q = MatrixX<double>::Zero(2, 2);
      VectorX<double> b = VectorX<double>::Zero(2);
      Q(0, 0) = 2;
      b(1) = -1;
      prog.AddQuadraticConstraint(Q, b, 0, 0, {t1[t], t2[t]});
    }
    // t1[t](0) * t2[t](0) == t3[t](0)
    {
      MatrixX<double> Q = MatrixX<double>::Zero(3, 3);
      VectorX<double> b = VectorX<double>::Zero(3);
      Q(0, 1) = 1;
      Q(1, 0) = 1;
      b(2) = -1;
      prog.AddQuadraticConstraint(Q, b, 0, 0, {t1[t], t2[t], t3[t]});
    }

    prog.AddLinearConstraint(t1[t](0) >= 0.2);
    prog.AddLinearConstraint(t2[t](0) >= 0.04);
    prog.AddLinearConstraint(t3[t](0) >= 0.008);

    for (int r = 0; r < num_rows; r++) {
      for (int c = 0; c < num_cols; c++) {
        c3[t][r][c] = prog.NewContinuousVariables(
            1, "c3[" + std::to_string(t) + "](" + std::to_string(r) + "," +
                   std::to_string(c) + ")");
        c2[t][r][c] = prog.NewContinuousVariables(
            1, "c2[" + std::to_string(t) + "](" + std::to_string(r) + "," +
                   std::to_string(c) + ")");
        c1[t][r][c] = prog.NewContinuousVariables(
            1, "c1[" + std::to_string(t) + "](" + std::to_string(r) + "," +
                   std::to_string(c) + ")");

        guess_coeffs = initial_guess.getPolynomial(t, r, c).GetCoefficients();
        const double c1_i = guess_coeffs(1);
        const double c2_i = guess_coeffs(2);
        const double c3_i = guess_coeffs(3);
        prog.SetInitialGuess(c1[t][r][c], Vector1<double>(c1_i));
        prog.SetInitialGuess(c2[t][r][c], Vector1<double>(c2_i));
        prog.SetInitialGuess(c3[t][r][c], Vector1<double>(c3_i));

        // q(1)
        /*
           q[t + 1](r, c) ==
           c3[t][r][c](0) * t3[t](0) +
           c2[t][r][c](0) * t2[t](0) +
           c1[t][r][c](0) * t1[t](0) +
           c0[t][r][c](0));
        */
        {
          MatrixX<double> Q = MatrixX<double>::Zero(6, 6);
          VectorX<double> b = VectorX<double>::Zero(6);
          Q(2, 5) = 1;
          Q(1, 4) = 1;
          Q(0, 3) = 1;
          Q(5, 2) = 1;
          Q(4, 1) = 1;
          Q(3, 0) = 1;
          prog.AddQuadraticConstraint(
              Q, b, q[t + 1](r, c) - q[t](r, c), q[t + 1](r, c) - q[t](r, c),
              {c1[t][r][c], c2[t][r][c], c3[t][r][c], t1[t], t2[t], t3[t]});
        }

        if (t == 0) {
          // First knot.
          prog.AddLinearEqualityConstraint(c1[t][r][c](0) == v0(r, c));
        } else {
          // Interior
          // v(0) continuous
          /*
             c1[t][r][c](0) ==
             3 * c3[t - 1][r][c](0) * t2[t - 1](0) +
             2 * c2[t - 1][r][c](0) * t1[t - 1](0) +
             c1[t - 1][r][c](0));
          */
          {
            MatrixX<double> Q = MatrixX<double>::Zero(6, 6);
            VectorX<double> b = VectorX<double>::Zero(6);
            Q(2, 4) = 2;
            Q(3, 5) = 3;
            Q(4, 2) = 2;
            Q(5, 3) = 3;
            b(0) = -1;
            b(1) = 1;
            prog.AddQuadraticConstraint(
                Q, b, 0, 0, {c1[t][r][c], c1[t - 1][r][c], c2[t - 1][r][c],
                             c3[t - 1][r][c], t1[t - 1], t2[t - 1]});
          }

          // vd(0) continuous
          /*
              c2[t][r][c](0) ==
              3 * c3[t - 1][r][c](0) * t1[t - 1](0) +
              c2[t - 1][r][c](0));
          */
          {
            MatrixX<double> Q = MatrixX<double>::Zero(4, 4);
            VectorX<double> b = VectorX<double>::Zero(4);
            Q(2, 3) = 3;
            Q(3, 2) = 3;
            b(0) = -1;
            b(1) = 1;
            prog.AddQuadraticConstraint(
                Q, b, 0, 0,
                {c2[t][r][c], c2[t - 1][r][c], c3[t - 1][r][c], t1[t - 1]});
          }
        }

        if (t == num_segments - 1) {
          // Last knot.
          /*
              v1(r, c) ==
              3 * c3[t][r][c](0) * t2[t](0) +
              2 * c2[t][r][c](0) * t1[t](0) +
              c1[t][r][c](0));
          */
          {
            MatrixX<double> Q = MatrixX<double>::Zero(5, 5);
            VectorX<double> b = VectorX<double>::Zero(5);
            Q(2, 4) = 3;
            Q(1, 3) = 2;
            Q(4, 2) = 3;
            Q(3, 1) = 2;
            b(0) = 1;
            auto con = prog.AddQuadraticConstraint(
                Q, b, v1(r, c), v1(r, c),
                {c1[t][r][c], c2[t][r][c], c3[t][r][c], t1[t], t2[t]});
            VectorX<double> x(5);
            x << c1_i, c2_i, c3_i, t_i, t_i * t_i;
            DRAKE_DEMAND(con.constraint()->CheckSatisfied(x));
          }
        }

        // v bounds.
        {
          MatrixX<double> Q = MatrixX<double>::Zero(5, 5);
          VectorX<double> b = VectorX<double>::Zero(5);
          Q(2, 4) = 3;
          Q(1, 3) = 2;
          Q(4, 2) = 3;
          Q(3, 1) = 2;
          b(0) = 1;
          auto con = prog.AddQuadraticConstraint(
              Q, b, v_lower(r, c), v_upper(r, c),
              {c1[t][r][c], c2[t][r][c], c3[t][r][c], t1[t], t2[t]});
          VectorX<double> x(5);
          x << c1_i, c2_i, c3_i, t_i, t_i * t_i;
          DRAKE_DEMAND(con.constraint()->CheckSatisfied(x));
        }
        {
          auto con = prog.AddLinearConstraint(v_lower(r, c) <= c1[t][r][c](0));
          VectorX<double> x = Vector1<double>(c1_i);
          DRAKE_DEMAND(con.constraint()->CheckSatisfied(x));
        }
        {
          auto con = prog.AddLinearConstraint(v_upper(r, c) >= c1[t][r][c](0));
          VectorX<double> x = Vector1<double>(c1_i);
          DRAKE_DEMAND(con.constraint()->CheckSatisfied(x));
        }

        {
          MatrixX<double> Q = MatrixX<double>::Zero(3, 3);
          VectorX<double> b = VectorX<double>::Zero(3);
          Q(1, 1) = -2;
          Q(0, 2) = 6;
          b(2) = -3 * v_upper(r, c);
          auto con = prog.AddQuadraticConstraint(
              Q, b, -INFINITY, 0, {c1[t][r][c], c2[t][r][c], c3[t][r][c]});
          VectorX<double> x(3);
          x << c1_i, c2_i, c3_i;
          std::cout << "x " << x.transpose() << ", u: " << v_upper(r, c)
                    << " haha: " << 0.5 * x.transpose() * Q * x + x.dot(b)
                    << "\n";
          std::cout << "v " << 0.5 * x.transpose() * Q * x / (3. * c3_i)
                    << "\n";
          DRAKE_DEMAND(con.constraint()->CheckSatisfied(x));

          b(2) = -3 * v_lower(r, c);
          auto con1 = prog.AddQuadraticConstraint(
              Q, b, 0, INFINITY, {c1[t][r][c], c2[t][r][c], c3[t][r][c]});
          DRAKE_DEMAND(con.constraint()->CheckSatisfied(x));
        }
        /*
        */

        // vd bounds.
        {
          MatrixX<double> Q = MatrixX<double>::Zero(3, 3);
          VectorX<double> b = VectorX<double>::Zero(3);
          Q(1, 2) = 6;
          Q(2, 1) = 6;
          b(0) = 2;
          auto con =
              prog.AddQuadraticConstraint(Q, b, vd_lower(r, c), vd_upper(r, c),
                                          {c2[t][r][c], c3[t][r][c], t1[t]});
          VectorX<double> x(3);
          x << c2_i, c3_i, t_i;
          DRAKE_DEMAND(con.constraint()->CheckSatisfied(x));
        }

        {
          auto con =
              prog.AddLinearConstraint(vd_lower(r, c) <= 2 * c2[t][r][c](0));
          VectorX<double> x = Vector1<double>(c2_i);
          DRAKE_DEMAND(con.constraint()->CheckSatisfied(x));
        }

        {
          auto con =
              prog.AddLinearConstraint(vd_upper(r, c) >= 2 * c2[t][r][c](0));
          VectorX<double> x = Vector1<double>(c2_i);
          DRAKE_DEMAND(con.constraint()->CheckSatisfied(x));
        }
      }
    }
  }

  // Add cost.
  symbolic::Expression cost = t1[0](0);
  for (int t = 1; t < num_segments; t++) {
    cost = cost + t1[t](0);
  }
  prog.AddLinearCost(cost);

  // solvers::GurobiSolver solver;
  // solvers::SolutionResult result = solver.Solve(prog);
  solvers::SolutionResult result = prog.Solve();
  prog.PrintSolution();
  DRAKE_DEMAND(result == solvers::SolutionResult::kSolutionFound);

  std::vector<PiecewisePolynomial<double>::PolynomialMatrix> polynomials(
      num_segments);
  std::vector<double> times(num_segments + 1, 0);
  for (int t = 0; t < num_segments; t++) {
    for (int r = 0; r < num_rows; r++) {
      for (int c = 0; c < num_cols; c++) {
        Vector4<double> coeff;
        coeff << q[t](r, c), prog.GetSolution(c1[t][r][c]),
            prog.GetSolution(c2[t][r][c]), prog.GetSolution(c3[t][r][c]);
        /*
        std::cout << "V0: " << prog.GetSolution(c1[t][r][c]) << "\n";
        std::cout << "V1: " << 3 * prog.GetSolution(c3[t][r][c]) *
                                   prog.GetSolution(t2[t]) +
                                   2 * prog.GetSolution(c2[t][r][c]) *
                                        prog.GetSolution(t1[t]) +
                                        prog.GetSolution(c1[t][r][c]) << "\n";
        std::cout << "VD0: " << 2 * prog.GetSolution(c2[t][r][c]) << "\n";
        std::cout << "VD1: " << 6 * prog.GetSolution(c3[t][r][c]) *
        prog.GetSolution(t1[t])(0) + 2 * prog.GetSolution(c2[t][r][c]) << "\n";
        */

        polynomials[t].resize(num_rows, num_cols);
        polynomials[t](r, c) =
            PiecewisePolynomial<double>::PolynomialType(coeff);
      }
    }
    times[t + 1] = times[t] + prog.GetSolution(t1[t])(0);
  }

  return PiecewisePolynomial<double>(polynomials, times);
}

}  // namespace jjz
}  // namespace drake
