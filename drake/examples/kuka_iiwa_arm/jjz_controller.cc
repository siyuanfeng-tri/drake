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
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

#include "drake/manipulation/planner/jacobian_ik.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

const char* const kLcmStatusChannel = "IIWA_STATUS";
const char* const kLcmCommandChannel = "IIWA_COMMAND";
// const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";

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
        cache_(iiwa.CreateKinematicsCache()),
        q_(VectorX<double>::Zero(iiwa.get_num_positions())),
        v_(VectorX<double>::Zero(iiwa.get_num_velocities())),
        ext_trq_(VectorX<double>::Zero(iiwa.get_num_actuators())) {
    DRAKE_DEMAND(iiwa.get_num_positions() == iiwa.get_num_velocities());
    DRAKE_DEMAND(iiwa.get_num_actuators() == iiwa.get_num_velocities());
  }

  void UpdateState(const lcmt_iiwa_status& msg) {
    // Check msg.
    DRAKE_DEMAND(msg.num_joints == iiwa_.get_num_positions());

    const double cur_time = msg.utime / 1e6;

    // Do velocity update first.
    if (init_) {
      // TODO need to filter.
      const double dt = cur_time - time_;
      for (int i = 0; i < msg.num_joints; ++i) {
        v_[i] = (msg.joint_position_measured[i] - q_[i]) / dt;
      }
    } else {
      v_.setZero();
    }

    // Update time, position, and torque.
    time_ = cur_time;
    for (int i = 0; i < msg.num_joints; ++i) {
      q_[i] = msg.joint_position_measured[i];
      ext_trq_[i] = msg.joint_torque_external[i];
    }

    // Update kinematics.
    cache_.initialize(q_, v_);
    iiwa_.doKinematics(cache_);

    init_ = true;
  }

  const KinematicsCache<double>& get_cache() const { return cache_; }
  const VectorX<double>& get_q() const { return q_; }
  const VectorX<double>& get_v() const { return v_; }
  const VectorX<double>& get_ext_trq() const { return ext_trq_; }
  double get_time() const { return time_; }

 private:
  const RigidBodyTree<double>& iiwa_;
  KinematicsCache<double> cache_;
  double time_{kUninitTime};

  VectorX<double> q_;
  VectorX<double> v_;
  VectorX<double> ext_trq_;

  bool init_{false};
};

class RobotPlanRunner {
 public:
  RobotPlanRunner(const std::string& model_path,
                  const std::string& end_effector_name,
                  const Isometry3<double>& X_WB)
      : jaco_planner_(model_path, end_effector_name, X_WB),
        robot_(jaco_planner_.get_robot()),
        state_(robot_) {
    VerifyIiwaTree(robot_);
    lcm_.subscribe(kLcmStatusChannel, &RobotPlanRunner::HandleStatus, this);
    // lcm_.subscribe(kLcmPlanChannel,
    //                &RobotPlanRunner::HandlePlan, this);
  }

  PiecewisePolynomial<double> SplineToDesiredConfiguration(
      const VectorX<double>& q_d, double duration) const {
    std::vector<double> times = {state_.get_time(), state_.get_time() + duration};
    std::vector<MatrixX<double>> knots = {state_.get_q(), q_d};
    MatrixX<double> zero = MatrixX<double>::Zero(robot_.get_num_velocities(), 1);
    return PiecewisePolynomial<double>::Cubic(times, knots, zero, zero);
  }

  void GenerateEETraj(const VectorX<double>& q, double r, double period, std::vector<double>* times, std::vector<Isometry3<double>>* poses) const {
    double dt = 1e-3;
    const int N = std::ceil(period / dt);
    times->resize(N);
    poses->resize(N);

    KinematicsCache<double> cache = robot_.CreateKinematicsCache();
    cache.initialize(q);
    robot_.doKinematics(cache);
    Isometry3<double> X_WE0 = robot_.CalcBodyPoseInWorldFrame(cache,
        jaco_planner_.get_end_effector());

    for (int i = 0; i < N; ++i) {
      (*times)[i] = (i + 1) * dt;
      (*poses)[i] = X_WE0;
      (*poses)[i].translation()[0] -= r * (std::cos(2 * M_PI * times->at(i) / period) - 1);
      (*poses)[i].translation()[1] += r * std::sin(2 * M_PI * times->at(i) / period);
    }
  }

  void Run() {
    lcmt_iiwa_command iiwa_command;
    iiwa_command.num_joints = robot_.get_num_positions();
    iiwa_command.joint_position.resize(robot_.get_num_positions(), 0.);
    iiwa_command.num_torques = 0;
    iiwa_command.joint_torque.resize(robot_.get_num_positions(), 0.);

    double cmd_time = kUninitTime;

    constexpr int GOTO = 0;
    constexpr int CIRCLE = 1;
    constexpr int NOOP = 2;
    constexpr int FB = 3;
//    constexpr int WAIT = 3;

    int STATE = GOTO;
    bool state_init = true;
    double state_t0;

    // traj for GOTO state.
    PiecewisePolynomial<double> traj;

    // traj for circle state.
    Isometry3<double> X_WE0;
    std::vector<double> times;
    std::vector<Isometry3<double>> poses;
    std::vector<VectorX<double>> q_sol;
    size_t q_ctr = 0;

    VectorX<double> q_nominal = robot_.getZeroConfiguration();

    KinematicsCache<double> cc = robot_.CreateKinematicsCache();

    while (true) {
      // Call lcm handle until at least one status message is
      // processed.
      while (0 == lcm_.handleTimeout(10) || state_.get_time() == cmd_time) {
      }

      switch (STATE) {
        // go home.
        case GOTO: {
          // make a spline to reset to home.
          if (state_init) {
            VectorX<double> q1 = robot_.getZeroConfiguration();
            q1[1] += 45. * M_PI / 180.;
            q1[3] -= M_PI / 2.;
            q1[5] += 45. * M_PI / 180.;
            traj = SplineToDesiredConfiguration(q1, 2);
            state_init = false;
            state_t0 = state_.get_time();
          }

          q_d_ = traj.value(state_.get_time());

          if (state_.get_time() - state_t0 > 2.1) {
            STATE = FB;
            state_init = true;
          }

          break;
        }

        case CIRCLE: {
          if (state_init) {
            VectorX<double> q1 = robot_.getZeroConfiguration();
            q1[1] += 45. * M_PI / 180.;
            q1[3] -= M_PI / 2.;
            q1[5] += 45. * M_PI / 180.;

            GenerateEETraj(q1, 0.1, 1.0, &times, &poses);
            jaco_planner_.Plan(q1, times, poses, q_nominal, &q_sol);
            q_ctr = 0;

            state_init = false;
            state_t0 = state_.get_time();
          }

          q_d_ = q_sol[q_ctr++];

          if (q_ctr >= q_sol.size()) {
            STATE = NOOP;
            state_init = true;
          }

          break;
        }

        case FB: {
          if (state_init) {
            VectorX<double> q1 = robot_.getZeroConfiguration();
            q1[1] += 45. * M_PI / 180.;
            q1[3] -= M_PI / 2.;
            q1[5] += 45. * M_PI / 180.;

            GenerateEETraj(q1, 0.1, 1.0, &times, &poses);
            q_ctr = 0;
            cc.initialize(state_.get_q());
            robot_.doKinematics(cc);

            state_init = false;
            state_t0 = state_.get_time();
          }

          const auto& pose_d = poses[q_ctr++];
          double dt = 1e-3;

          Isometry3<double> X_WE =
              robot_.CalcBodyPoseInWorldFrame(cc,
                  jaco_planner_.get_end_effector());

          Vector6<double> V_WE_d =
              jaco_planner_.ComputePoseDiffInWorldFrame(X_WE, pose_d) / dt;
          VectorX<double> v = jaco_planner_.ComputeDofVelocity(cc, V_WE_d, q_nominal, dt);
          q_d_ = cc.getQ() + v * dt;
          cc.initialize(q_d_);
          robot_.doKinematics(cc);

          if (q_ctr >= poses.size()) {
            q_ctr = 0;
            //STATE = NOOP;
            //state_init = true;
          }
        }

        case NOOP: {
          break;
        }
      }

      // Make msg.
      iiwa_command.utime = static_cast<int64_t>(state_.get_time() * 1e6);
      for (int i = 0; i < robot_.get_num_positions(); i++) {
        iiwa_command.joint_position[i] = q_d_[i];
      }

      // Send command
      lcm_.publish(kLcmCommandChannel, &iiwa_command);
      // Log time of command.
      cmd_time = state_.get_time();
    }
  }

 private:
  void HandleStatus(const lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_iiwa_status* status) {
    // iiwa_status_ = *status;
    state_.UpdateState(*status);
  }

  lcm::LCM lcm_;
  manipulation::planner::JacobianIk jaco_planner_;
  const RigidBodyTree<double>& robot_;
  // lcmt_iiwa_status iiwa_status_;
  IiwaState state_;
  VectorX<double> q_d_;
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
