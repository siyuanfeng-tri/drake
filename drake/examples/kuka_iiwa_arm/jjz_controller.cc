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
// const char* const kLcmJjzControllerDebug = "CTRL_DEBUG";

const std::string kPath =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";
const Isometry3<double> kBaseOffset = Isometry3<double>::Identity();

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
    iiwa_command.num_torques = 0;
    iiwa_command.joint_torque.resize(robot_.get_num_positions(), 0.);

    iiwa_status_.utime = -1;
    bool first_tick = true;
    jjz::IiwaState state(robot_);

    VectorX<double> q_cmd(7);
    lcmt_jjz_controller ctrl_debug{};
    ctrl_debug.wall_time = -1;

    // traj for GOTO state.
    PiecewisePolynomial<double> traj;

    // traj for cartesian mode.
    manipulation::PiecewiseCartesianTrajectory<double> ee_traj;

    // Starting point.
    Vector3<double> x_GQ(0, 0, M_PI / 2.);
    Isometry3<double> X_GQ = Isometry3<double>::Identity();
    X_GQ.linear() =
        AngleAxis<double>(x_GQ[2], Vector3<double>::UnitZ()).toRotationMatrix();
    X_GQ.translation() = Vector3<double>(x_GQ[0], x_GQ[1], 0);

    // ee_traj = jjz::PlanPlanarPushingTrajMultiAction(x_GQ, 5);
    ee_traj = jjz::PlanPlanarPushingTrajMultiAction(x_GQ, 5, "graph.txt");
    VectorX<double> q1 = jjz::PointIk(jjz::X_WG * ee_traj.get_pose(0), frame_T_,
                                      (RigidBodyTree<double>*)&robot_);

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

      // Initialize command to measured q.
      if (first_tick) {
        for (int i = 0; i < robot_.get_num_positions(); i++) {
          q_cmd[i] = iiwa_status_.joint_position_measured[i];
        }
        first_tick = false;

        // Make initial plan to go to q1.
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
        // Servo + traj follow after go_to_q1 is done.
        if (plan->get_name().compare("go_to_q1") == 0) {
          VectorX<double> q_nominal = robot_.getZeroConfiguration();
          jjz::MoveToolFollowTraj* new_plan = new jjz::MoveToolFollowTraj(
              "pushing", &robot_, q1, q_nominal, ee_traj);
          new_plan->set_integrating(true);
          new_plan->set_f_W_d(Vector3<double>(0, 0, 1));
          new_plan->set_ki_force(Vector3<double>::Constant(0.00001));
          new_plan->set_f_W_dead_zone(Vector3<double>(INFINITY, INFINITY, 0));
          new_plan->set_position_int_max_range(Vector3<double>::Constant(0.2));
          plan.reset(new_plan);
        }
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
      auto tmp = jjz::pose_to_vec(X_WT);
      eigenVectorToCArray(tmp, ctrl_debug.X_WE);

      eigenVectorToCArray(state.get_ext_wrench(), ctrl_debug.ext_wrench);
      eigenVectorToCArray(state.get_q(), ctrl_debug.q0);
      eigenVectorToCArray(q_cmd, ctrl_debug.q1);
      lcm_.publish(kLcmJjzControllerDebug, &ctrl_debug);
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
