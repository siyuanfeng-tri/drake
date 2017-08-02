#include <fstream>
#include <iostream>
#include <memory>

#include <lcm/lcm-cpp.hpp>
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_jjz_controller.hpp"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

#include "drake/examples/PlanarPushing/dubins_interface.h"
#include "drake/examples/PlanarPushing/pushing_multi_actions.h"

#include "drake/manipulation/planner/jacobian_ik.h"
#include "drake/manipulation/util/trajectory_utils.h"
#include "drake/util/drakeUtil.h"
#include "drake/util/lcmUtil.h"

#include "drake/math/rotation_matrix.h"
#include "drake/multibody/constraint/rigid_body_constraint.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/rigid_body_ik.h"

#include "drake/examples/kuka_iiwa_arm/jjz_common.h"
#include "drake/examples/kuka_iiwa_arm/jjz_primitives.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

const char* const kLcmStatusChannel = "IIWA_STATUS";
const char* const kLcmCommandChannel = "IIWA_COMMAND";
const char* const kLcmJjzControllerDebug = "CTRL_DEBUG";

const std::string kPath =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";
const Isometry3<double> kBaseOffset = Isometry3<double>::Identity();

class RobotPlanRunner {
 public:
  RobotPlanRunner(const RigidBodyTree<double>& robot)
      : jaco_planner_(&robot),
        robot_(robot),
        frame_T_("tool", robot_.FindBody(jjz::kEEName), jjz::X_ET) {
    VerifyIiwaTree(robot_);
    lcm::Subscription* sub =
        lcm_.subscribe(kLcmStatusChannel, &RobotPlanRunner::HandleStatus, this);
    // THIS IS VERY IMPORTANT!!
    sub->setQueueCapacity(1);
  }

  PiecewisePolynomial<double> SplineToDesiredConfiguration(
      const VectorX<double>& q0, const VectorX<double>& q_d, double t0,
      double duration) const {
    std::vector<double> times = {t0, t0 + duration};
    std::vector<MatrixX<double>> knots = {q0, q_d};
    MatrixX<double> zero =
        MatrixX<double>::Zero(robot_.get_num_velocities(), 1);
    return PiecewisePolynomial<double>::Cubic(times, knots, zero, zero);
  }

  VectorX<double> PointIk(const Isometry3<double>& X_WE) const {
    std::cout << "PointIk: desired pose:\n" << X_WE.matrix() << "\n\n";

    std::vector<RigidBodyConstraint*> constraint_array;
    RigidBodyTree<double>* cao_robot = (RigidBodyTree<double>*)&robot_;

    IKoptions ikoptions(cao_robot);

    Vector3<double> pos_tol(0.001, 0.001, 0.001);
    double rot_tol = 0.001;
    Vector3<double> pos_lb = X_WE.translation() - pos_tol;
    Vector3<double> pos_ub = X_WE.translation() + pos_tol;

    WorldPositionConstraint pos_con(
        cao_robot, frame_T_.get_rigid_body().get_body_index(),
        Vector3<double>::Zero(), pos_lb, pos_ub, Vector2<double>::Zero());

    constraint_array.push_back(&pos_con);

    // Adds a rotation constraint.
    WorldQuatConstraint quat_con(
        cao_robot, frame_T_.get_rigid_body().get_body_index(),
        math::rotmat2quat(X_WE.linear()), rot_tol, Vector2<double>::Zero());
    constraint_array.push_back(&quat_con);

    VectorX<double> q_res = VectorX<double>::Zero(7);
    VectorX<double> zero = VectorX<double>::Zero(7);
    VectorX<double> q_ini = zero;
    q_ini[1] = 45. * M_PI / 180;
    q_ini[3] = -90. * M_PI / 180;
    q_ini[5] = 45. * M_PI / 180;

    int info;
    std::vector<std::string> infeasible_constraints;
    inverseKin(cao_robot, q_ini, zero, constraint_array.size(),
               constraint_array.data(), ikoptions, &q_res, &info,
               &infeasible_constraints);

    DRAKE_DEMAND(info == 1);
    return q_res;
  }

  /*
  manipulation::PiecewiseCartesianTrajectory<double> GenerateToolTraj(
      const VectorX<double>& q, double r, double period, double dt) const {
    const int N = std::ceil(period / dt);

    KinematicsCache<double> cache = robot_.CreateKinematicsCache();
    cache.initialize(q);
    robot_.doKinematics(cache);
    Isometry3<double> X_WT0 = robot_.CalcFramePoseInWorldFrame(cache, frame_T_);

    std::vector<double> times(N);
    std::vector<MatrixX<double>> pos(N);
    eigen_aligned_std_vector<Quaternion<double>> rot(N);

    for (int i = 0; i < N; ++i) {
      times[i] = (i + 1) * dt;
      pos[i] = X_WT0.translation();
      pos[i](0, 0) -= r * (std::cos(2 * M_PI * times[i] / period) - 1);
      pos[i](1, 0) += r * std::sin(2 * M_PI * times[i] / period);

      rot[i] = Quaternion<double>(X_WT0.linear());
    }
    PiecewiseQuaternionSlerp<double> rot_traj(times, rot);
    PiecewisePolynomial<double> pos_traj =
        PiecewisePolynomial<double>::FirstOrderHold(times, pos);
    return manipulation::PiecewiseCartesianTrajectory<double>(pos_traj,
                                                              rot_traj);
  }
  */

  // Returned stuff is in ZZJ's Goal frame. 0 is origin of Goal frame.
  manipulation::PiecewiseCartesianTrajectory<double>
  PlanPlanarPushingTrajMultiAction(const Vector3<double>& x_GQ, double duration,
                                   std::string load_file_name = "") const {
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
      // jjz
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
    std::vector<MatrixX<double>> pos(total_way_points,
                                     MatrixX<double>::Zero(3, 1));
    eigen_aligned_std_vector<Quaternion<double>> rot(total_way_points);

    double cur_time = 0.0;
    int index = 0;
    for (int id_traj = 0; id_traj < num_action_segs; ++id_traj) {
      Eigen::Matrix<double, Eigen::Dynamic, 3> pusher_poses =
          all_pusher_poses[id_traj];
      int num_way_points = pusher_poses.rows();
      for (int i = 0; i < num_way_points; ++i) {
        cur_time = cur_time + dt;
        times[index] = cur_time;
        pos[index](0, 0) = pusher_poses(i, 0);
        pos[index](1, 0) = pusher_poses(i, 1);
        // std::cout << index << " : " << cur_time <<"," << pos[index](0, 0) <<
        // "," << pos[index](1, 0) << std::endl;
        // std::cout << "jjz: pusher pose" << pusher_poses.row(i) << "\n";
        // std::cout << "jjz: object pose" << object_poses.row(i) << "\n";

        // sfeng thinks jjz's angle is somehow 90 deg off from me.
        std::cout << "t: " << cur_time << ", pose " << pusher_poses.row(i)
                  << "\n";
        Matrix3<double> X_GT(AngleAxis<double>(pusher_poses(i, 2) + M_PI / 2.,
                                               Vector3<double>::UnitZ()));
        rot[index] = Quaternion<double>(X_GT);
        ++index;
      }
      if (id_traj < num_action_segs - 1 &&
          action_id[id_traj] != action_id[id_traj + 1]) {
        // The robot first moves up.
        cur_time = cur_time + time_lift_up;
        times[index] = cur_time;
        pos[index] = pos[index - 1];
        // Add z value.
        pos[index](2, 0) = dist_lift_up;
        rot[index] = rot[index - 1];
        index++;
        // The robot then moves to the plane above the next pushing location and
        // align with the initial pose of the next trajectory.
        cur_time = cur_time + time_move_above;
        times[index] = cur_time;
        Eigen::Vector3d nxt_push_pose =
            all_pusher_poses[id_traj + 1].row(0).transpose();
        pos[index](0, 0) = nxt_push_pose(0);
        pos[index](1, 0) = nxt_push_pose(1);
        pos[index](2, 0) = dist_lift_up;
        std::cout << "t: " << cur_time << ", pose " << nxt_push_pose.transpose()
                  << "\n";
        Matrix3<double> X_GT(AngleAxis<double>(nxt_push_pose(2) + M_PI / 2.,
                                               Vector3<double>::UnitZ()));
        rot[index] = Quaternion<double>(X_GT);
        index++;
        // The robot then moves down to the next pushing location.
        cur_time = cur_time + time_move_down;
        times[index] = cur_time;
        pos[index](0, 0) = nxt_push_pose(0);
        pos[index](1, 0) = nxt_push_pose(1);
        rot[index] = rot[index - 1];
        index++;
      }
    }

    std::cout << "i tot" << index << " " << total_way_points << "\n";

    PiecewiseQuaternionSlerp<double> rot_traj(times, rot);
    PiecewisePolynomial<double> pos_traj =
        PiecewisePolynomial<double>::FirstOrderHold(times, pos);
    manipulation::PiecewiseCartesianTrajectory<double> traj(pos_traj, rot_traj);

    return traj;
  }

  /*
  manipulation::PiecewiseCartesianTrajectory<double> PlanPlanarPushingTraj(
      const Isometry3<double>& pose0, double duration) const {
    // Limit surface A_11.
    double ls_a = 1.1;
    // Limit surface A_33 / rho^2
    double ls_b = 4500;
    // Coefficient of contact friction
    double mu = 0.15;

    // local frame.
    Vector2<double> pt(-0.02, 0);
    Vector2<double> normal(1, 0);

    DubinsPushPlanner planner(pt, normal, mu, ls_a, ls_b);

    Vector3<double> start_pose(0, 0, 0);
    Vector3<double> goal_pose(0.2, 0.15, M_PI / 2.);

    // Are these sampled uniformly?
    int num_way_points = 100;
    Eigen::Matrix<double, Eigen::Dynamic, 3> object_poses;
    Eigen::Matrix<double, Eigen::Dynamic, 3> pusher_poses;
    planner.PlanPath(start_pose, goal_pose, num_way_points, &object_poses,
                     &pusher_poses);

    double dt = duration / (num_way_points - 1);
    std::vector<double> times(num_way_points);
    std::vector<MatrixX<double>> pos(num_way_points);
    eigen_aligned_std_vector<Quaternion<double>> rot(num_way_points);

    std::cout << "psoe0 " << pose0.translation().transpose() << "\n";

    for (int i = 0; i < num_way_points; ++i) {
      times[i] = (i + 1) * dt;

      pos[i] = pose0.translation();
      pos[i](0, 0) += object_poses(i, 0);
      pos[i](1, 0) += object_poses(i, 1);
      // std::cout << "jjz: pusher pose" << pusher_poses.row(i) << "\n";
      std::cout << "jjz: object pose" << object_poses.row(i) << "\n";

      // sfeng thinks jjz's angle is somehow 90 deg off from me.
      Matrix3<double> X_WT(
          AngleAxis<double>(object_poses(i, 2), Vector3<double>::UnitZ()));
      Matrix3<double> X_WE = X_WT * jjz::R_ET.transpose();
      rot[i] = Quaternion<double>(X_WE);
    }

    PiecewiseQuaternionSlerp<double> rot_traj(times, rot);
    PiecewisePolynomial<double> pos_traj =
        PiecewisePolynomial<double>::FirstOrderHold(times, pos);
    manipulation::PiecewiseCartesianTrajectory<double> traj(pos_traj, rot_traj);

    return traj;
  }
  */

  Eigen::Matrix<double, 7, 1> pose_to_vec(const Isometry3<double>& pose) const {
    Eigen::Matrix<double, 7, 1> ret;
    ret.head<3>() = pose.translation();

    /*
    ret.segment<3>(3) = math::rotmat2rpy(pose.linear());
    */
    Quaternion<double> quat(pose.linear());
    ret[3] = quat.w();
    ret[4] = quat.x();
    ret[5] = quat.y();
    ret[6] = quat.z();

    return ret;
  }

  void Run() {
    lcmt_iiwa_command iiwa_command;
    iiwa_command.num_joints = robot_.get_num_positions();
    iiwa_command.joint_position.resize(robot_.get_num_positions(), 0.);
    iiwa_command.num_torques = 0;
    iiwa_command.joint_torque.resize(robot_.get_num_positions(), 0.);

    iiwa_status_.utime = -1;
    bool first_tick = true;
    jjz::IiwaState state(robot_);

    constexpr int GOTO = 0;
    constexpr int HOME = 1;
    constexpr int JACOBI = 2;
    constexpr int FORCE_SERVO = 3;

    int STATE = GOTO;
    bool state_init = true;
    double state_t0;
    double control_dt;
    VectorX<double> q_cmd(7);
    lcmt_jjz_controller ctrl_debug{};
    ctrl_debug.wall_time = -1;

    lcmt_iiwa_status msg{};

    // traj for GOTO state.
    PiecewisePolynomial<double> traj;

    // traj for cartesian mode.
    manipulation::PiecewiseCartesianTrajectory<double> ee_traj;
    VectorX<double> q_nominal = robot_.getZeroConfiguration();
    KinematicsCache<double> cc = robot_.CreateKinematicsCache();

    // Starting point.
    Vector3<double> x_GQ(0, 0, M_PI / 2.);
    Isometry3<double> X_GQ = Isometry3<double>::Identity();
    X_GQ.linear() =
        AngleAxis<double>(x_GQ[2], Vector3<double>::UnitZ()).toRotationMatrix();
    X_GQ.translation() = Vector3<double>(x_GQ[0], x_GQ[1], 0);

    // ee_traj = PlanPlanarPushingTrajMultiAction(x_GQ, 5);
    ee_traj = PlanPlanarPushingTrajMultiAction(x_GQ, 5, "graph.txt");
    VectorX<double> q1 =
        PointIk(jjz::X_WG * ee_traj.get_pose(0) * jjz::X_ET.inverse());

    // VERY IMPORTANT HACK
    for (int i = 0; i < 5; i++) {
      while (0 >= lcm_.handleTimeout(10) || iiwa_status_.utime == -1) {
      }
    }

    Isometry3<double> X_WT_d, X_WT;
    double wall_clock0 = get_time();

    std::unique_ptr<jjz::FSMState> plan;
    while (true) {
      // Call lcm handle until at least one status message is
      // processed.
      while (0 >= lcm_.handleTimeout(10) || iiwa_status_.utime == -1) {
      }

      DRAKE_DEMAND(state.UpdateState(iiwa_status_));

      // Initialize command to measured q.
      if (first_tick) {
        for (int i = 0; i < robot_.get_num_positions(); i++) {
          q_cmd[i] = iiwa_status_.joint_position_measured[i];
        }
        first_tick = false;

        plan.reset(new jjz::MoveJoint("go_to_q1", q1, 3.));
      }

      if (!plan->is_init()) {
        plan->Initialize(state);
      }
      plan->Update(state);
      plan->Control(state, q_cmd);

      // send command
      iiwa_command.utime = static_cast<int64_t>(state.get_time() * 1e6);
      for (int i = 0; i < robot_.get_num_positions(); i++) {
        iiwa_command.joint_position[i] = q_cmd[i];
      }
      lcm_.publish(kLcmCommandChannel, &iiwa_command);

      // state transition.
      if (plan->IsDone(state)) {
        if (plan->get_name().compare("go_to_q1") == 0) {
          jjz::MoveToolFollowTraj* new_plan = new jjz::MoveToolFollowTraj(
              "pushing", &robot_, q1, q_nominal, ee_traj);
          // new_plan->set_integrating(true);
          new_plan->set_f_W_d(Vector3<double>(0, 0, 1));
          new_plan->set_ki_force(Vector3<double>::Constant(0.00001));
          new_plan->set_f_W_dead_zone(Vector3<double>(INFINITY, INFINITY, 0));
          new_plan->set_position_int_max_range(Vector3<double>::Constant(0.2));
          plan.reset(new_plan);
        }
      }
    }

    exit(-1);

    while (true) {
      // Call lcm handle until at least one status message is
      // processed.
      while (0 >= lcm_.handleTimeout(10) || iiwa_status_.utime == -1) {
      }

      // No new states.
      DRAKE_DEMAND(state.UpdateState(iiwa_status_));
      control_dt = state.get_dt();

      // Initialize command to measured q.
      if (first_tick) {
        for (int i = 0; i < robot_.get_num_positions(); i++) {
          q_cmd[i] = iiwa_status_.joint_position_measured[i];
        }
        first_tick = false;
      }

      std::cout << "f_ext: " << state.get_ext_wrench().transpose() << "\n";

      switch (STATE) {
        case GOTO: {
          // make a spline to reset to home.
          if (state_init) {
            traj = SplineToDesiredConfiguration(state.get_q(), q1,
                                                state.get_time(), 5);
            state_init = false;
            state_t0 = state.get_time();
            std::cout << "GOTO start: " << state_t0 << "\n";
          }

          q_cmd = traj.value(state.get_time());

          if (state.get_time() - state_t0 > 5.1) {
            STATE = FORCE_SERVO;
            // STATE = JACOBI;
            state_init = true;
          }

          break;
        }

        case HOME: {
          // make a spline to reset to home.
          if (state_init) {
            VectorX<double> q0 = robot_.getZeroConfiguration();
            traj = SplineToDesiredConfiguration(state.get_q(), q0,
                                                state.get_time(), 3);
            state_init = false;
            state_t0 = state.get_time();
            std::cout << "HOME start: " << state_t0 << "\n";
          }

          q_cmd = traj.value(state.get_time());

          if (state.get_time() - state_t0 > 3.4) {
            STATE = GOTO;
            state_init = true;
          }

          break;
        }

        case JACOBI: {
          // const double period = 5;
          if (state_init) {
            state_init = false;
            state_t0 = state.get_time();
            std::cout << "PUSHING start: " << state_t0 << "\n";

            cc.initialize(q1);
            robot_.doKinematics(cc);
          }

          double interp_t = state.get_time() - state_t0;
          // double interp_t = std::fmod((state.get_time() - state_t0), period);
          const Isometry3<double> X_GT = ee_traj.get_pose(interp_t);

          X_WT_d = jjz::X_WG * X_GT;
          X_WT = robot_.CalcFramePoseInWorldFrame(cc, frame_T_);

          Vector6<double> V_WT_d =
              jaco_planner_.ComputePoseDiffInWorldFrame(X_WT, X_WT_d) /
              control_dt;

          Vector6<double> gain_T = Vector6<double>::Constant(1);
          gain_T(1) = 0;  // no pitch tracking.
          VectorX<double> v = jaco_planner_.ComputeDofVelocity(
              cc, frame_T_, V_WT_d, q_nominal, control_dt, gain_T);
          cc.initialize(cc.getQ() + v * control_dt);
          robot_.doKinematics(cc);

          q_cmd = cc.getQ();

          /*
          auto tmp = pose_to_vec(X_WE_d);
          eigenVectorToCArray(tmp, ctrl_debug.X_WE_d);
          Isometry3<double> aa = Isometry3<double>::Identity();
          aa.linear() = R_ET;
          tmp = pose_to_vec(X_WE * aa);
          eigenVectorToCArray(tmp, ctrl_debug.X_WE_ik);
          */

          break;
        }

        case FORCE_SERVO: {
          if (state_init) {
            state_init = false;
            state_t0 = state.get_time();
            std::cout << "FORCE_SERVO start: " << state_t0 << "\n";

            cc.initialize(q1);
            robot_.doKinematics(cc);

            X_WT_d = robot_.CalcFramePoseInWorldFrame(cc, frame_T_);
          }

          double state_t = state.get_time() - state_t0;
          X_WT = robot_.CalcFramePoseInWorldFrame(cc, frame_T_);

          double force_i_gain = 0.00001;
          double force_min_range = 5;

          if (state_t > 1) {
            // Just the force part.
            Vector3<double> f_W_d = Vector3<double>::Zero();
            f_W_d[2] = 10;

            // Measure force
            Vector3<double> f_W = state.get_ext_wrench().tail<3>();

            Vector3<double> f_diff = f_W_d - f_W;
            for (int i = 0; i < 3; i++) {
              if (std::abs(f_diff[i]) < force_min_range) {
                f_diff[i] = 0;
              }
            }

            // notice the -
            X_WT_d.translation() -= force_i_gain * f_diff;

            auto tmp = pose_to_vec(X_WT_d);
            eigenVectorToCArray(tmp, ctrl_debug.X_WE_d);
            tmp = pose_to_vec(X_WT);
            eigenVectorToCArray(tmp, ctrl_debug.X_WE_d);

            Vector6<double> V_WT_d =
                jaco_planner_.ComputePoseDiffInWorldFrame(X_WT, X_WT_d) /
                control_dt;
            VectorX<double> v = jaco_planner_.ComputeDofVelocity(
                cc, frame_T_, V_WT_d, q_nominal, control_dt);
            cc.initialize(cc.getQ() + v * control_dt);
            robot_.doKinematics(cc);

            q_cmd = cc.getQ();
          }

          break;
        }
      }

      double wall_clock = get_time() - wall_clock0;
      // Make cmd msg.
      iiwa_command.utime = static_cast<int64_t>(state.get_time() * 1e6);
      // iiwa_command.wall_time = static_cast<int64_t>(wall_clock * 1e6);
      for (int i = 0; i < robot_.get_num_positions(); i++) {
        iiwa_command.joint_position[i] = q_cmd[i];
      }
      lcm_.publish(kLcmCommandChannel, &iiwa_command);

      // Make debug msg.
      ctrl_debug.utime = static_cast<int64_t>(state.get_time() * 1e6);
      if (ctrl_debug.wall_time == -1) {
        ctrl_debug.wall_dt = 0;
      } else {
        ctrl_debug.wall_dt = wall_clock - (ctrl_debug.wall_time / 1e6);
      }
      ctrl_debug.wall_time = static_cast<int64_t>(wall_clock * 1e6);
      ctrl_debug.dt = control_dt;

      Isometry3<double> X_WT =
          robot_.CalcFramePoseInWorldFrame(state.get_cache(), frame_T_);
      auto tmp = pose_to_vec(X_WT);
      eigenVectorToCArray(tmp, ctrl_debug.X_WE);

      eigenVectorToCArray(state.get_ext_wrench(), ctrl_debug.ext_wrench);
      eigenVectorToCArray(state.get_q(), ctrl_debug.q0);
      eigenVectorToCArray(q_cmd, ctrl_debug.q1);
      lcm_.publish(kLcmJjzControllerDebug, &ctrl_debug);
    }
  }

 private:
  void HandleStatus(const lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_iiwa_status* status) {
    SetStateMsg(*status);
  }

  lcmt_iiwa_status CopyStateMsg() const {
    std::lock_guard<std::mutex> guard(state_lock_);
    return iiwa_status_;
  }

  void SetStateMsg(const lcmt_iiwa_status& msg) {
    std::lock_guard<std::mutex> guard(state_lock_);
    iiwa_status_ = msg;
  }

  lcm::LCM lcm_;
  manipulation::planner::JacobianIk jaco_planner_;
  const RigidBodyTree<double>& robot_;
  const RigidBodyFrame<double> frame_T_;

  mutable std::mutex state_lock_;
  lcmt_iiwa_status iiwa_status_{};

  mutable std::mutex cmd_lock_;
  VectorX<double> q_d_;
};

int do_main() {
  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      kPath, drake::multibody::joints::kFixed, nullptr, &tree);
  RobotPlanRunner runner(tree);
  runner.Run();
  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main() { return drake::examples::kuka_iiwa_arm::do_main(); }
