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

#include "drake/manipulation/planner/jacobian_ik.h"
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

using manipulation::PiecewiseCartesianTrajectory;

class RobotPlanRunner {
 public:
  RobotPlanRunner(const RigidBodyTree<double>& robot)
      : robot_(robot),
        frame_T_("tool", robot_.FindBody(jjz::kEEName), jjz::X_ET) {
    VerifyIiwaTree(robot_);
    lcm::Subscription* sub =
        lcm_.subscribe(kLcmStatusChannel, &RobotPlanRunner::HandleStatus, this);
    // THIS IS VERY IMPORTANT!!
    sub->setQueueCapacity(1);
  }

  void Run() {
    lcmt_iiwa_command iiwa_command;
    iiwa_command.num_joints = robot_.get_num_positions();
    iiwa_command.joint_position.resize(robot_.get_num_positions(), 0.);
    iiwa_command.num_torques = robot_.get_num_positions();
    iiwa_command.joint_torque.resize(robot_.get_num_positions(), 0.);

    iiwa_status_.utime = -1;
    bool first_tick = true;
    jjz::IiwaState state(robot_, frame_T_);

    VectorX<double> q_cmd(7);
    VectorX<double> trq_cmd = VectorX<double>::Zero(7);

    // traj for GOTO state.
    PiecewisePolynomial<double> traj;

    // traj for cartesian mode.
    manipulation::PiecewiseCartesianTrajectory<double> X_WT_traj;

    // Starting point.
    Vector3<double> x_GQ(0, 0, M_PI / 2.);
    Isometry3<double> X_GQ = Isometry3<double>::Identity();
    X_GQ.linear() =
        AngleAxis<double>(x_GQ[2], Vector3<double>::UnitZ()).toRotationMatrix();
    X_GQ.translation() = Vector3<double>(x_GQ[0], x_GQ[1], 0);

    // X_WT_traj = jjz::PlanPlanarPushingTrajMultiAction(x_GQ, jjz::X_WG, 5);

    /*
    X_WT_traj =
        jjz::PlanPlanarPushingTrajMultiAction(x_GQ, jjz::X_WG, 5, "graph.txt");
    VectorX<double> q1 = jjz::PointIk(Eigen::Translation<double, 3>(Vector3<double>(0, 0, 0.05)) * X_WT_traj.get_pose(0), frame_T_,
                                      (RigidBodyTree<double>*)&robot_);
    */
    VectorX<double> q1 = jjz::PointIk(
        Eigen::Translation<double, 3>(Vector3<double>(0, 0, 0.05)) * jjz::X_WG,
        frame_T_, (RigidBodyTree<double>*)&robot_);

    // VERY IMPORTANT HACK, To make sure that no stale messages are in the
    // queue.
    for (int i = 0; i < 5; i++) {
      while (0 >= lcm_.handleTimeout(10) || iiwa_status_.utime == -1) {
      }
    }

    // Main loop.
    std::unique_ptr<jjz::FSMState> plan;
    while (true) {
      // Call lcm handle until at least one status message is
      // processed.
      while (0 >= lcm_.handleTimeout(10) || iiwa_status_.utime == -1) {
      }

      DRAKE_DEMAND(state.UpdateState(iiwa_status_));
      lcmt_jjz_controller ctrl_debug{};
      jjz::FillDebugMessage(state, &ctrl_debug);

      // Initialize command to measured q.
      if (first_tick) {
        for (int i = 0; i < robot_.get_num_positions(); i++) {
          q_cmd[i] = iiwa_status_.joint_position_measured[i];
        }
        first_tick = false;

        // Make initial plan to go to q1.
        plan.reset(new jjz::MoveJoint("go_to_q1", state.get_q(), q1, 3.));
      }

      if (!plan->is_init()) {
        plan->Initialize(state);
      }
      plan->Update(state, &ctrl_debug);
      plan->Control(state, q_cmd, trq_cmd, &ctrl_debug);

      // send command
      iiwa_command.utime = static_cast<int64_t>(state.get_time() * 1e6);
      for (int i = 0; i < robot_.get_num_positions(); i++) {
        iiwa_command.joint_position[i] = q_cmd[i];
        iiwa_command.joint_torque[i] = trq_cmd[i];
      }
      lcm_.publish(kLcmCommandChannel, &iiwa_command);

      // send debug msg.
      lcm_.publish(kLcmJjzControllerDebug, &ctrl_debug);

      // state transition.
      if (plan->IsDone(state)) {
        if (plan->get_name().compare("go_to_q1") == 0) {
          jjz::MoveToolStraightUntilTouch* new_plan =
              new jjz::MoveToolStraightUntilTouch("go_down",
                  &robot_, &frame_T_, q1, Vector3<double>(0, 0, -1), 0.03);
          plan.reset(new_plan);
        }
        /* else if (plan->get_name().compare("go_down") == 0) {
          jjz::HoldPositionAndApplyForce* new_plan =
              new jjz::HoldPositionAndApplyForce("hold",
                  &robot_, &frame_T_);
          Vector6<double> wrench = Vector6<double>::Zero();
          // wrench[4] = -10;
          wrench[5] = 100;
          new_plan->set_desired_ext_wrench(wrench);
          plan.reset(new_plan);
        }
        */
        else if (plan->get_name().compare("go_down") == 0) {
          auto cache = robot_.CreateKinematicsCache();
          cache.initialize(q_cmd);
          robot_.doKinematics(cache);
          Isometry3<double> X_WT0 =
              robot_.CalcFramePoseInWorldFrame(cache, frame_T_);

          Isometry3<double> X_WT1 =
              Eigen::Translation<double, 3>(Vector3<double>(0, -0.2, 0)) *
              X_WT0 * AngleAxis<double>(M_PI / 4., Vector3<double>::UnitZ());

          auto traj = PiecewiseCartesianTrajectory<double>::
              MakeCubicLinearWithEndLinearVelocity({0.5, 3}, {X_WT0, X_WT1},
                                                   Vector3<double>::Zero(),
                                                   Vector3<double>::Zero());

          jjz::MoveToolFollowTraj* new_plan = new jjz::MoveToolFollowTraj(
              "go_right", &robot_, &frame_T_, q_cmd, traj);
          new_plan->set_mu(1.);
          new_plan->set_fz(10);

          plan.reset(new_plan);
        }

        /*
        // Servo + traj follow after go_to_q1 is done.
        if (plan->get_name().compare("go_to_q1") == 0) {
          jjz::MoveToolFollowTraj* new_plan = new jjz::MoveToolFollowTraj(
              "pushing", &robot_, q1, X_WT_traj);
          new_plan->set_integrating(true);
          new_plan->set_f_W_d(Vector3<double>(0, 0, 1));
          new_plan->set_ki_force(Vector3<double>::Constant(0.00001));
          new_plan->set_f_W_dead_zone(Vector3<double>(INFINITY, INFINITY, 0));
          new_plan->set_position_int_max_range(Vector3<double>::Constant(0.2));
          plan.reset(new_plan);
        }
        */

        /*
        // go straight dodwn.
        if (plan->get_name().compare("go_to_q1") == 0) {
          auto cache = robot_.CreateKinematicsCache();
          cache.initialize(q1);
          robot_.doKinematics(cache);
          Isometry3<double> X_WT0 =
              robot_.CalcFramePoseInWorldFrame(cache, frame_T_);
          Isometry3<double> X_WT1 = X_WT0;
          X_WT1.translation()[2] -= 0.05;

          auto traj = PiecewiseCartesianTrajectory<double>::
              MakeCubicLinearWithEndLinearVelocity({0, 2}, {X_WT0, X_WT1},
                                                   Vector3<double>::Zero(),
                                                   Vector3<double>::Zero());

          jjz::MoveToolFollowTraj* new_plan = new jjz::MoveToolFollowTraj(
              "move_straight_down", &robot_, q1, traj);

          new_plan->set_force_servo(true);
          new_plan->set_f_W_d(Vector3<double>(0, 0, 15));
          new_plan->set_ki_force(Vector3<double>::Constant(0.00001));
          new_plan->set_f_W_dead_zone(Vector3<double>(INFINITY, INFINITY, 5));
          new_plan->set_position_int_max_range(Vector3<double>::Constant(0.2));
          plan.reset(new_plan);
        } else if (plan->get_name().compare("move_straight_down") == 0 ||
                   plan->get_name().compare("drag_right") == 0) {
          jjz::MoveToolFollowTraj* move_plan =
              dynamic_cast<jjz::MoveToolFollowTraj*>(plan.get());
          // Reset the traj from the current ik's T frame to some random thing.
          Isometry3<double> X_WT0 = move_plan->get_X_WT_ik();
          Isometry3<double> X_WT1 =
              Eigen::Translation<double, 3>(Vector3<double>(0, -0.2, 0)) *
              X_WT0 * AngleAxis<double>(M_PI / 4., Vector3<double>::UnitZ());
          auto traj = PiecewiseCartesianTrajectory<double>::
              MakeCubicLinearWithEndLinearVelocity({0, 2}, {X_WT0, X_WT1},
                                                   Vector3<double>::Zero(),
                                                   Vector3<double>::Zero());
          move_plan->set_X_WT_traj(traj);

          // Need to clear the integrator.
          move_plan->reset_pose_integrator();
          move_plan->set_name("drag_left");
          // Reset the timer.
          move_plan->Initialize(state);
        } else if (plan->get_name().compare("drag_left") == 0) {
          jjz::MoveToolFollowTraj* move_plan =
              dynamic_cast<jjz::MoveToolFollowTraj*>(plan.get());
          // Reset the traj from the current ik's T frame to some random thing.
          Isometry3<double> X_WT0 = move_plan->get_X_WT_ik();
          Isometry3<double> X_WT1 =
              Eigen::Translation<double, 3>(Vector3<double>(0, 0.2, 0)) *
              X_WT0 * AngleAxis<double>(-M_PI / 4., Vector3<double>::UnitZ());
          auto traj = PiecewiseCartesianTrajectory<double>::
              MakeCubicLinearWithEndLinearVelocity({0, 2}, {X_WT0, X_WT1},
                                                   Vector3<double>::Zero(),
                                                   Vector3<double>::Zero());
          move_plan->set_X_WT_traj(traj);

          // Need to clear the integrator.
          move_plan->reset_pose_integrator();
          move_plan->set_name("drag_right");
          // Reset the timer.
          move_plan->Initialize(state);
        }
        */
        // Others.
      }
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
  const RigidBodyTree<double>& robot_;
  const RigidBodyFrame<double> frame_T_;

  mutable std::mutex state_lock_;
  lcmt_iiwa_status iiwa_status_{};
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
