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

#include "drake/multibody/constraint/rigid_body_constraint.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/rigid_body_ik.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

const char* const kLcmStatusChannel = "IIWA_STATUS";
const char* const kLcmCommandChannel = "IIWA_COMMAND";

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
  RobotPlanRunner(const RigidBodyTree<double>& robot,
                  const std::string& end_effector_name,
                  const VectorX<double>& q_ini)
      : robot_(robot),
        end_effector_(*robot_.FindBody(end_effector_name)),
        state_(robot_),
        q_ini_(q_ini) {
    VerifyIiwaTree(robot_);
    lcm_.subscribe(kLcmStatusChannel, &RobotPlanRunner::HandleStatus, this);

    q_calib_ = GenerateCalibrateConfigurations(q_ini_);
  }

  PiecewisePolynomial<double> SplineToDesiredConfiguration(
      const VectorX<double>& q_d, double duration) const {
    std::vector<double> times = {state_.get_time(),
                                 state_.get_time() + duration};
    std::vector<MatrixX<double>> knots = {state_.get_q(), q_d};
    MatrixX<double> zero =
        MatrixX<double>::Zero(robot_.get_num_velocities(), 1);
    return PiecewisePolynomial<double>::Cubic(times, knots, zero, zero);
  }

  // Make a 3 x 3 grid of end effector pose.
  std::vector<Isometry3<double>> GenerateGridOfCalibrateEndEffectorPose(
      double dy, double dz, double rot_z, double rot_y) const {
    std::vector<Isometry3<double>> ret;

    // The rotation from the end effector frame to the last link frame =
    // [0, 0, -1;
    //  0, 1, 0;
    //  1, 0, 0];

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        Isometry3<double> pose = Isometry3<double>::Identity();
        pose.translation()[1] = (1 - j) * dy;
        pose.translation()[2] = (-1 + i) * dz;

        pose.linear() =
            (AngleAxis<double>((-1 + j) * rot_z, Vector3<double>::UnitZ()) *
             AngleAxis<double>((-1 + i) * rot_y, Vector3<double>::UnitY()))
                .toRotationMatrix();
        ret.push_back(pose);
      }
    }

    return ret;
  }

  std::vector<VectorX<double>> GenerateCalibrateConfigurations(
      const VectorX<double>& q_center) {
    std::vector<VectorX<double>> ret;

    KinematicsCache<double> cache = robot_.CreateKinematicsCache();
    // Do fk(q).
    cache.initialize(q_center);
    robot_.doKinematics(cache);
    Isometry3<double> X_WE_center =
        robot_.CalcBodyPoseInWorldFrame(cache, end_effector_);

    std::vector<Isometry3<double>> poses =
        GenerateGridOfCalibrateEndEffectorPose(0.1, 0.1, 0.2, 0.2);

    const Vector3<double> pos_tol(0.001, 0.001, 0.001);
    const double rot_tol = 0.01;

    // This is a hack.
    RigidBodyTree<double>* cao_robot = (RigidBodyTree<double>*)(&robot_);

    // For each pose, solve an ik, and save the q.
    for (const auto& pose : poses) {
      // End effector pose in the the world.
      Isometry3<double> X_WE = X_WE_center * pose;

      //////////////////////////////////////////////////////////
      std::vector<RigidBodyConstraint*> constraint_array;
      IKoptions ikoptions(cao_robot);

      Vector3<double> pos_lb = X_WE.translation() - pos_tol;
      Vector3<double> pos_ub = X_WE.translation() + pos_tol;

      WorldPositionConstraint pos_con(cao_robot, end_effector_.get_body_index(),
                                      Vector3<double>::Zero(), pos_lb, pos_ub,
                                      Vector2<double>::Zero());

      constraint_array.push_back(&pos_con);

      // Adds a rotation constraint.
      WorldQuatConstraint quat_con(cao_robot, end_effector_.get_body_index(),
                                   math::rotmat2quat(X_WE.linear()), rot_tol,
                                   Vector2<double>::Zero());
      constraint_array.push_back(&quat_con);

      VectorX<double> q_res = q_center;
      int info;
      std::vector<std::string> infeasible_constraints;
      inverseKin(cao_robot, q_center, q_center, constraint_array.size(),
                 constraint_array.data(), ikoptions, &q_res, &info,
                 &infeasible_constraints);
      //////////////////////////////////////////////////////////

      ret.push_back(q_res);

      std::cout << q_res.transpose() << "\n";
    }

    return ret;
  }

  void Run() {
    lcmt_iiwa_command iiwa_command;
    iiwa_command.num_joints = robot_.get_num_positions();
    iiwa_command.joint_position.resize(robot_.get_num_positions(), 0.);
    iiwa_command.num_torques = 0;
    iiwa_command.joint_torque.resize(robot_.get_num_positions(), 0.);

    double cmd_time = kUninitTime;

    constexpr int GOTO = 0;
    constexpr int WAIT = 1;
    constexpr int NOOP = 2;

    int STATE = GOTO;
    bool state_init = true;
    double state_t0;

    // traj for GOTO state.
    PiecewisePolynomial<double> traj;

    const double kTrajTime = 2;
    const double kWaitTime = 2;

    size_t calib_index = 0;

    while (true) {
      // Call lcm handle until at least one status message is
      // processed.
      while (0 == lcm_.handleTimeout(10) || state_.get_time() == cmd_time) {
      }

      switch (STATE) {
        case GOTO: {
          if (state_init) {
            traj = SplineToDesiredConfiguration(q_calib_[calib_index], kTrajTime);
            calib_index++;

            state_init = false;
            state_t0 = state_.get_time();
          }

          q_d_ = traj.value(state_.get_time());

          if (state_.get_time() - state_t0 > kTrajTime) {
            STATE = WAIT;
            state_init = true;
          }

          break;
        }

        case WAIT: {
          if (state_init) {
            state_init = false;
            state_t0 = state_.get_time();
          }

          if (state_.get_time() - state_t0 > kWaitTime) {
            if (calib_index >= q_calib_.size()) {
              STATE = NOOP;
            } else {
              STATE = GOTO;
            }
            state_init = true;
          }
          break;
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
  const RigidBodyTree<double>& robot_;
  const RigidBody<double>& end_effector_;
  // lcmt_iiwa_status iiwa_status_;
  IiwaState state_;
  VectorX<double> q_d_;

  VectorX<double> q_ini_;
  std::vector<VectorX<double>> q_calib_;
};

int do_main() {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow(kPath), multibody::joints::kFixed, tree.get());


  // Joint angles for the "center" pose, jjz need to update this.
  VectorX<double> q_center = VectorX<double>::Zero(7);
  q_center[0] = -5 / 180. * M_PI;
  q_center[1] = 40. / 180. * M_PI;
  q_center[3] = -90. / 180. * M_PI;
  q_center[5] = -45. / 180. * M_PI;

  RobotPlanRunner runner(*tree, kEEName, q_center);
  runner.Run();
  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main() { return drake::examples::kuka_iiwa_arm::do_main(); }
