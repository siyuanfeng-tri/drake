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
#include "drake/manipulation/util/trajectory_utils.h"
#include "drake/util/drakeUtil.h"
#include "drake/util/lcmUtil.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

const char* const kLcmStatusChannel = "IIWA_STATUS";
const char* const kLcmCommandChannel = "IIWA_COMMAND";
const char* const kLcmJjzControllerDebug = "CTRL_DEBUG";

constexpr double kUninitTime = -1.0;
const std::string kPath =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";
const std::string kEEName = "iiwa_link_ee";
const Isometry3<double> kBaseOffset = Isometry3<double>::Identity();

class IiwaState {
 public:
  IiwaState(const RigidBodyTree<double>& iiwa)
      : iiwa_(iiwa),
        end_effector_(*iiwa_.FindBody(kEEName)),
        cache_(iiwa.CreateKinematicsCache()),
        q_(VectorX<double>::Zero(iiwa.get_num_positions())),
        v_(VectorX<double>::Zero(iiwa.get_num_velocities())),
        trq_(VectorX<double>::Zero(iiwa.get_num_actuators())),
        ext_trq_(VectorX<double>::Zero(iiwa.get_num_actuators())) {
    DRAKE_DEMAND(iiwa.get_num_positions() == iiwa.get_num_velocities());
    DRAKE_DEMAND(iiwa.get_num_actuators() == iiwa.get_num_velocities());
  }

  bool UpdateState(const lcmt_iiwa_status& msg) {
    // Check msg.
    DRAKE_DEMAND(msg.num_joints == iiwa_.get_num_positions());

    const double cur_time = msg.utime / 1e6;
    // Same time stamp, should just return.
    if (init_ && cur_time == time_)
      return false;

    // Do velocity update first.
    if (init_) {
      // TODO need to filter.
      delta_time_ = cur_time - time_;
      for (int i = 0; i < msg.num_joints; ++i) {
        v_[i] = (msg.joint_position_measured[i] - q_[i]) / delta_time_;
      }
    } else {
      delta_time_ = 0;
      v_.setZero();
    }

    // Update time, position, and torque.
    time_ = cur_time;
    for (int i = 0; i < msg.num_joints; ++i) {
      q_[i] = msg.joint_position_measured[i];
      trq_[i] = msg.joint_torque_measured[i];
      ext_trq_[i] = msg.joint_torque_external[i];
    }

    // Update kinematics.
    cache_.initialize(q_, v_);
    iiwa_.doKinematics(cache_);

    J_ = iiwa_.CalcBodySpatialVelocityJacobianInWorldFrame(
        cache_, end_effector_);
    ext_wrench_ = J_.transpose().colPivHouseholderQr().solve(ext_trq_);

    init_ = true;

    return true;
  }

  const KinematicsCache<double>& get_cache() const { return cache_; }
  const VectorX<double>& get_q() const { return q_; }
  const VectorX<double>& get_v() const { return v_; }
  const VectorX<double>& get_ext_trq() const { return ext_trq_; }
  const VectorX<double>& get_trq() const { return trq_; }
  const Vector6<double>& get_ext_wrench() const { return ext_wrench_; }
  double get_time() const { return time_; }
  double get_dt() const { return delta_time_; }

 private:
  const RigidBodyTree<double>& iiwa_;
  const RigidBody<double>& end_effector_;
  KinematicsCache<double> cache_;
  double time_{kUninitTime};
  double delta_time_{0};

  VectorX<double> q_;
  VectorX<double> v_;
  VectorX<double> trq_;
  VectorX<double> ext_trq_;
  MatrixX<double> J_;

  // J^T * F = ext_trq_
  Vector6<double> ext_wrench_;

  bool init_{false};
};

class RobotPlanRunner {
 public:
  RobotPlanRunner(const std::string& model_path,
                  const std::string& end_effector_name,
                  const Isometry3<double>& X_WB)
      : jaco_planner_(model_path, end_effector_name, X_WB),
        robot_(jaco_planner_.get_robot()),
        q_d_(VectorX<double>::Zero(robot_.get_num_positions())) {
    VerifyIiwaTree(robot_);
    lcm_.subscribe(kLcmStatusChannel, &RobotPlanRunner::HandleStatus, this);
  }

  PiecewisePolynomial<double> SplineToDesiredConfiguration(
      const VectorX<double>& q0, const VectorX<double>& q_d,
      double t0, double duration) const {
    std::vector<double> times = {t0, t0 + duration};
    std::vector<MatrixX<double>> knots = {q0, q_d};
    MatrixX<double> zero =
        MatrixX<double>::Zero(robot_.get_num_velocities(), 1);
    return PiecewisePolynomial<double>::Cubic(times, knots, zero, zero);
  }

  manipulation::PiecewiseCartesianTrajectory<double> GenerateEETraj(
      const VectorX<double>& q, double r, double period, double dt) const {
    const int N = std::ceil(period / dt);

    KinematicsCache<double> cache = robot_.CreateKinematicsCache();
    cache.initialize(q);
    robot_.doKinematics(cache);
    Isometry3<double> X_WE0 = robot_.CalcBodyPoseInWorldFrame(
        cache, jaco_planner_.get_end_effector());

    std::vector<double> times(N);
    std::vector<MatrixX<double>> pos(N);
    eigen_aligned_std_vector<Quaternion<double>> rot(N);

    for (int i = 0; i < N; ++i) {
      times[i] = (i + 1) * dt;
      pos[i] = X_WE0.translation();
      pos[i](0, 0) -=
          r * (std::cos(2 * M_PI * times[i] / period) - 1);
      pos[i](1, 0) +=
          r * std::sin(2 * M_PI * times[i]/ period);

      rot[i] = Quaternion<double>(X_WE0.linear());
    }
    PiecewiseQuaternionSlerp<double> rot_traj(times, rot);
    PiecewisePolynomial<double> pos_traj =
        PiecewisePolynomial<double>::FirstOrderHold(times, pos);
    return manipulation::PiecewiseCartesianTrajectory<double>(pos_traj, rot_traj);
  }

  Eigen::Matrix<double, 7, 1> pose_to_vec(const Isometry3<double>& pose) const {
    Eigen::Matrix<double, 7, 1> ret;
    ret.head<3>() = pose.translation();
    Quaternion<double> quat(pose.linear());
    ret[3] = quat.w();
    ret[4] = quat.x();
    ret[5] = quat.y();
    ret[6] = quat.z();

    return ret;
  }

  void ControlThread() {
    IiwaState state(robot_);
    lcmt_iiwa_status msg;

    constexpr int GOTO = 0;
    constexpr int HOME = 1;
    // constexpr int CIRCLE = 1;
    constexpr int NOOP = 2;
    constexpr int JACOBI = 3;
    // constexpr int WAIT = 3;

    int STATE = GOTO;
    bool state_init = true;
    double state_t0;
    double control_dt;

    // traj for GOTO state.
    PiecewisePolynomial<double> traj;

    // traj for cartesian mode.
    manipulation::PiecewiseCartesianTrajectory<double> ee_traj;

    VectorX<double> q_nominal = robot_.getZeroConfiguration();
    KinematicsCache<double> cc = robot_.CreateKinematicsCache();

    VectorX<double> q_cmd;

    lcmt_jjz_controller ctrl_debug{};

    while (true) {
      // TODO lock this
      {
        std::lock_guard<std::mutex> guard(state_lock_);
        msg = iiwa_status_;
      }

      // No status yet.
      if (msg.utime == -1)
        continue;

      if (!state.UpdateState(msg))
        continue;

      control_dt = state.get_dt();

      switch (STATE) {
        // go home.
        case GOTO: {
          // make a spline to reset to home.
          if (state_init) {
            VectorX<double> q1 = robot_.getZeroConfiguration();
            q1[1] += 45. * M_PI / 180.;
            q1[3] -= M_PI / 2.;
            q1[5] += 45. * M_PI / 180.;
            traj = SplineToDesiredConfiguration(state.get_q(), q1, state.get_time(), 2);
            state_init = false;
            state_t0 = state.get_time();
          }

          q_cmd = traj.value(state.get_time());

          if (state.get_time() - state_t0 > 2.1) {
            STATE = JACOBI;
            state_init = true;
          }

          break;
        }

        case HOME: {
          // make a spline to reset to home.
          if (state_init) {
            VectorX<double> q1 = robot_.getZeroConfiguration();
            traj = SplineToDesiredConfiguration(state.get_q(), q1, state.get_time(), 2);
            state_init = false;
            state_t0 = state.get_time();
          }

          q_cmd = traj.value(state.get_time());

          if (state.get_time() - state_t0 > 2.1) {
            STATE = GOTO;
            state_init = true;
          }

          break;
        }

        case JACOBI: {
          const double period = 2;
          const double radius = 0.1;
          if (state_init) {
            VectorX<double> q1 = robot_.getZeroConfiguration();
            q1[1] += 45. * M_PI / 180.;
            q1[3] -= M_PI / 2.;
            q1[5] += 45. * M_PI / 180.;

            double dt = 3e-3;
            ee_traj = GenerateEETraj(q1, radius, period, dt);
            cc.initialize(state.get_q());
            robot_.doKinematics(cc);

            state_init = false;
            state_t0 = state.get_time();
          }

          double interp_t = std::fmod((state.get_time() - state_t0), period);
          const Isometry3<double> pose_d = ee_traj.get_pose(interp_t);

          Isometry3<double> X_WE = robot_.CalcBodyPoseInWorldFrame(
              cc, jaco_planner_.get_end_effector());

          Vector6<double> V_WE_d =
              jaco_planner_.ComputePoseDiffInWorldFrame(X_WE, pose_d) / control_dt;
          VectorX<double> v =
              jaco_planner_.ComputeDofVelocity(cc, V_WE_d, q_nominal, control_dt);
          cc.initialize(cc.getQ() + v * control_dt);
          robot_.doKinematics(cc);

          q_cmd = cc.getQ();

          auto tmp = pose_to_vec(pose_d);
          eigenVectorToCArray(tmp, ctrl_debug.X_WE_d);
          eigenVectorToCArray(state.get_ext_wrench(), ctrl_debug.ext_wrench);

          break;
        }

        case NOOP: {
          break;
        }
      }

      // make msg.
      ctrl_debug.utime = static_cast<int64_t>(state.get_time() * 1e6);
      ctrl_debug.dt = control_dt;
      eigenVectorToCArray(state.get_q(), ctrl_debug.q0);
      eigenVectorToCArray(q_cmd, ctrl_debug.q1);
      lcm_.publish(kLcmJjzControllerDebug, &ctrl_debug);

      // TODO lock this:
      {
        std::lock_guard<std::mutex> guard(cmd_lock_);
        q_d_ = q_cmd;
      }
    }
  }

  void Run() {
    lcmt_iiwa_command iiwa_command;
    iiwa_command.num_joints = robot_.get_num_positions();
    iiwa_command.joint_position.resize(robot_.get_num_positions(), 0.);
    iiwa_command.num_torques = 0;
    iiwa_command.joint_torque.resize(robot_.get_num_positions(), 0.);

    iiwa_status_.utime = -1;
    int64_t cmd_time = -1;

    std::thread control_thread =
        std::thread(&RobotPlanRunner::ControlThread, this);

    while (true) {
      // Call lcm handle until at least one status message is
      // processed.
      while (0 == lcm_.handleTimeout(10) || iiwa_status_.utime == cmd_time) {
      }

      // Initialize q_d_ to measured q.
      if (cmd_time == -1) {
        for (int i = 0; i < robot_.get_num_positions(); i++) {
          q_d_[i] = iiwa_status_.joint_position_measured[i];
        }
      }

      // Make msg.
      iiwa_command.utime = iiwa_status_.utime;
      {
        std::lock_guard<std::mutex> guard(cmd_lock_);
        for (int i = 0; i < robot_.get_num_positions(); i++) {
          // TODO lock this.
          iiwa_command.joint_position[i] = q_d_[i];
        }
      }

      // Send command
      lcm_.publish(kLcmCommandChannel, &iiwa_command);
      // Log time of command.
      cmd_time = iiwa_status_.utime;
    }
  }

 private:
  void HandleStatus(const lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_iiwa_status* status) {
    std::lock_guard<std::mutex> guard(state_lock_);
    iiwa_status_ = *status;
  }

  lcm::LCM lcm_;
  manipulation::planner::JacobianIk jaco_planner_;
  const RigidBodyTree<double>& robot_;
  lcmt_iiwa_status iiwa_status_{};
  // IiwaState state_;
  VectorX<double> q_d_;

  std::mutex state_lock_;
  std::mutex cmd_lock_;
};

int do_main() {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  RobotPlanRunner runner(kPath, kEEName, kBaseOffset);
  runner.Run();
  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main() { return drake::examples::kuka_iiwa_arm::do_main(); }
