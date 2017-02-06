/// @file
///
/// Generates a canned IK demo plan for an iiwa arm starting from the
/// zero configuration and sends that plan over lcm using the
/// robot_plan_t message.

#include <iostream>
#include <memory>

#include <lcm/lcm-cpp.hpp>

#include "bot_core/robot_state_t.hpp"
#include "drake/examples/kuka_iiwa_arm/kuka_ik_planner.h"
#include "drake/common/drake_path.h"
#include "drake/lcmt_iiwa_status.hpp"

#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/util/lcmUtil.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";

class Action {
 public:
  Action() {
    iiwa_time_ = -1;
    wsg_time_ = -1;
    obj_time_ = -1;
    act_start_time_ = -1;
    iiwa_q_ = VectorX<double>::Zero(7);
    iiwa_v_ = VectorX<double>::Zero(7);
    obj_pose_ = Isometry3<double>::Identity();
    wsg_q_ = 0;
    wsg_force_ = 0;
  }

  virtual void HandleIiwaStatus(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const lcmt_iiwa_status* iiwa_msg) {
    bool is_first_msg = iiwa_time_ == -1;
    double cur_time = iiwa_msg->utime / 1e6;
    double dt = cur_time - iiwa_time_;

    iiwa_time_ = cur_time;

    if (is_first_msg) {
      for (int i = 0; i < iiwa_msg->num_joints; i++) {
        iiwa_q_[i] = iiwa_msg->joint_position_measured[i];
      }
    }

    for (int i = 0; i < iiwa_msg->num_joints; i++) {
      // Need to LP
      iiwa_v_[i] = (iiwa_msg->joint_position_measured[i] - iiwa_q_[i]) / dt;
      iiwa_q_[i] = iiwa_msg->joint_position_measured[i];
    }
  }

  virtual void HandleWsgStatus(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const lcmt_schunk_wsg_status* wsg_msg) {
    bool is_first_msg = wsg_time_ == -1;
    double cur_time = wsg_msg->utime / 1e6;
    double dt = cur_time - wsg_time_;

    wsg_time_ = cur_time;

    if (is_first_msg) {
      wsg_q_ = wsg_msg->actual_position_mm / 1000.;
    }

    // Need to LP
    wsg_v_ = (wsg_msg->actual_position_mm / 1000. - wsg_q_) / dt;
    wsg_q_ = wsg_msg->actual_position_mm / 1000.;
    wsg_force_ = wsg_msg->actual_force;
  }

  virtual void HandleObjectStatus(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const bot_core::robot_state_t* obj_msg) {
    obj_time_ = obj_msg->utime / 1e6;
    obj_pose_ = DecodePose(obj_msg->pose);
    obj_vel_ = DecodeTwist(obj_msg->twist);
  }

  virtual bool ActionFinished() const =0;
  virtual bool ActionFailed() const =0;

  virtual bool ActionStarted() const {
    if (act_start_time_ == -1)
      return false;
    return true;
  }

  double get_iiwa_time() const { return iiwa_time_; }

  double get_wsg_time() const { return wsg_time_; }

  double get_obj_time() const { return obj_time_; }

  const Isometry3<double>& get_object_pose() const { return obj_pose_; }

  const Vector6<double>& get_object_velocity() const { return obj_vel_; }

  virtual void Reset() {
    act_start_time_ = -1;
  }

 protected:
  double act_start_time_;
  double iiwa_time_;

  VectorX<double> iiwa_q_;
  VectorX<double> iiwa_v_;

  double wsg_time_;
  double wsg_q_; // units [m]
  double wsg_v_; // units [m]
  double wsg_force_;

  double obj_time_;
  Isometry3<double> obj_pose_;
  Vector6<double> obj_vel_;
};

class IiwaMoveCartesian : public Action {
 public:
  IiwaMoveCartesian(lcm::LCM* lcm, const std::string& channel, const std::string& iiwa_model_path)
      : planner_(iiwa_model_path), iiwa_(planner_.get_robot()), cache_(iiwa_.CreateKinematicsCache()), lcm_(lcm), channel_(channel) {}

  void HandleIiwaStatus(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const lcmt_iiwa_status* iiwa_msg) {
    Action::HandleIiwaStatus(rbuf, chan, iiwa_msg);
    cache_.initialize(iiwa_q_, iiwa_v_);
    iiwa_.doKinematics(cache_, true);
  }

  void MoveEnedEffectorThroughWaypoints(const std::vector<KukaIkPlanner::IkCartesianWaypoint>& waypoints) {
    act_start_time_ = iiwa_time_;
    waypoints_ = waypoints;
    planner_.PlanTrajectory(waypoints, VectorX<double>::Zero(7), &ik_res_);
    robotlocomotion::robot_plan_t plan = planner_.EncodeMessage(ik_res_);
    lcm_->publish(channel_, &plan);
  }

  void Reset() {
    Action::Reset();
    waypoints_.clear();
  }

  // TODO
  bool ActionFailed() const {
    return false;
  }

  bool ActionFinished() const {
    if (!ActionStarted())
      return false;

    if (iiwa_time_ - act_start_time_ > waypoints_.back().time && iiwa_v_.norm() < 1e-2) {
      return true;
    } else {
      return false;
    }
  }

 private:
  KukaIkPlanner planner_;
  KukaIkPlanner::IkResult ik_res_;
  const RigidBodyTree<double>& iiwa_;
  KinematicsCache<double> cache_;

  lcm::LCM* lcm_;
  const std::string channel_;

  std::vector<KukaIkPlanner::IkCartesianWaypoint> waypoints_;
};

class WsgAction : public Action {
 public:
  WsgAction(lcm::LCM* lcm, const std::string& channel) : lcm_(lcm), channel_(channel) {}

  void OpenGripper() {
    act_start_time_ = wsg_time_;
    lcmt_schunk_wsg_command msg;
    msg.target_position_mm = 110;
    lcm_->publish(channel_, &msg);
  }

  void CloseGripper() {
    act_start_time_ = wsg_time_;
    lcmt_schunk_wsg_command msg;
    msg.target_position_mm = 0;
    lcm_->publish(channel_, &msg);
  }

  // TODO
  bool ActionFailed() const {
    return false;
  }

  bool ActionFinished() const {
    if (!ActionStarted())
      return false;

    if (std::abs(wsg_v_) < 1e-2 && wsg_time_ - act_start_time_ > 0.5)
      return true;
    return false;
  }

 private:
  lcm::LCM* lcm_;
  const std::string channel_;
};

int main(int argc, const char* argv[]) {
  std::string path = GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf";
  lcm::LCM lcm;

  IiwaMoveCartesian iiwa_move(&lcm, kLcmPlanChannel, path);
  WsgAction wsg_act(&lcm, "SCHUNK_WSG_COMMAND");

  lcm.subscribe("IIWA_STATUS", &IiwaMoveCartesian::HandleIiwaStatus, &iiwa_move);
  lcm.subscribe("OBJECT_STATE_EST", &Action::HandleObjectStatus, (Action*)&iiwa_move);
  lcm.subscribe("SCHUNK_WSG_STATUS", &Action::HandleWsgStatus, (Action*)&wsg_act);
  lcm.subscribe("OBJECT_STATE_EST", &Action::HandleObjectStatus, (Action*)&wsg_act);

  int state = -1;

  int N = 3;
  double z_high = 0.3;
  double z_low = 0.1;
  double dz = (z_high - z_low) / N;
  double dt = 0.3;

  // lcm handle loop
  while (true) {
    // handle msg untill we have all msg from iiwa and gripper
    while (0 == lcm.handleTimeout(10) || iiwa_move.get_iiwa_time() == -1 || wsg_act.get_wsg_time() == -1 || iiwa_move.get_obj_time() == -1) { }

    switch (state) {
      case -1:
        if (!wsg_act.ActionStarted()) {
          wsg_act.OpenGripper();
        }

        if (wsg_act.ActionFinished()) {
          state = 0;
          wsg_act.Reset();
        }

      case 0:
        if (!iiwa_move.ActionStarted()) {
          std::vector<KukaIkPlanner::IkCartesianWaypoint> waypoints;
          KukaIkPlanner::IkCartesianWaypoint wp;
          wp.time = 2;
          std::cout << "obj: " << iiwa_move.get_object_pose().translation().transpose() << std::endl;
          wp.pose.translation() << 0.68, 0, z_high;
          wp.enforce_quat = true;
          waypoints.push_back(wp);

          iiwa_move.MoveEnedEffectorThroughWaypoints(waypoints);
          std::cout << "exe 0 move: " << iiwa_move.get_iiwa_time() << std::endl;
        }

        if (iiwa_move.ActionFinished()) {
          state = 1;
          iiwa_move.Reset();
        }

        break;

      case 1:
        if (!iiwa_move.ActionStarted()) {
          std::vector<KukaIkPlanner::IkCartesianWaypoint> waypoints;
          KukaIkPlanner::IkCartesianWaypoint wp;
          wp.time = 0;
          wp.pose.translation() << 0.68, 0, z_high;
          wp.enforce_quat = true;

          for (int i = 0; i < N; i++) {
            wp.time += dt;
            wp.pose.translation()[2] -= dz;
            waypoints.push_back(wp);
          }

          iiwa_move.MoveEnedEffectorThroughWaypoints(waypoints);
          std::cout << "exe 1 move: " << iiwa_move.get_iiwa_time() << std::endl;
        }

        if (iiwa_move.ActionFinished()) {
          state = 2;
          iiwa_move.Reset();
        }

        break;

      case 2:
        if (!wsg_act.ActionStarted()) {
          wsg_act.CloseGripper();
        }

        if (wsg_act.ActionFinished()) {
          state = 3;
          wsg_act.Reset();
        }
        break;

      case 3:
        if (!iiwa_move.ActionStarted()) {
          std::vector<KukaIkPlanner::IkCartesianWaypoint> waypoints;
          KukaIkPlanner::IkCartesianWaypoint wp;
          wp.time = 0;
          wp.pose.translation() << 0.68, 0, z_low;
          wp.enforce_quat = true;

          for (int i = 0; i < N; i++) {
            wp.time += dt;
            wp.pose.translation()[2] += dz;
            waypoints.push_back(wp);
          }

          iiwa_move.MoveEnedEffectorThroughWaypoints(waypoints);
          std::cout << "exe 2 move: " << iiwa_move.get_iiwa_time() << std::endl;
        }

        if (iiwa_move.ActionFinished()) {
          state = 4;
          iiwa_move.Reset();
        }
        break;

      case 4:
        if (!iiwa_move.ActionStarted()) {
          std::vector<KukaIkPlanner::IkCartesianWaypoint> waypoints;
          KukaIkPlanner::IkCartesianWaypoint wp;
          wp.time = 2;
          wp.pose.translation() << 0, 0.68, z_high;
          wp.pose.linear() = Matrix3<double>(AngleAxis<double>(M_PI / 2., Vector3<double>::UnitZ()));
          wp.enforce_quat = true;

          waypoints.push_back(wp);

          iiwa_move.MoveEnedEffectorThroughWaypoints(waypoints);
          std::cout << "exe 3 move: " << iiwa_move.get_iiwa_time() << std::endl;
        }

        if (iiwa_move.ActionFinished() &&
            iiwa_move.get_object_velocity().head<2>().norm() < 1e-2) {
          std::cout << "obj: " << iiwa_move.get_object_pose().translation().transpose() << std::endl;
          state = 5;
          iiwa_move.Reset();
        }
        break;

      case 5:
        if (!iiwa_move.ActionStarted()) {
          std::vector<KukaIkPlanner::IkCartesianWaypoint> waypoints;
          KukaIkPlanner::IkCartesianWaypoint wp;
          wp.time = 0;
          wp.pose.translation() << 0, 0.68, z_high;
          wp.pose.linear() = Matrix3<double>(AngleAxis<double>(M_PI / 2., Vector3<double>::UnitZ()));
          wp.enforce_quat = true;

          z_low = 0.1 + (0.2 - iiwa_move.get_object_pose().translation()[2] + 0.01);
          std::cout << "new z low: " << z_low << std::endl;
          dz = (z_high - z_low) / N;

          for (int i = 0; i < N; i++) {
            wp.time += dt;
            wp.pose.translation()[2] -= dz;
            std::cout << wp.pose.translation()[2] << std::endl;
            waypoints.push_back(wp);
          }

          iiwa_move.MoveEnedEffectorThroughWaypoints(waypoints);
          std::cout << "exe 4 move: " << iiwa_move.get_iiwa_time() << std::endl;
        }

        if (iiwa_move.ActionFinished()) {
          state = 6;
          iiwa_move.Reset();
        }
        break;

      case 6:
        if (!wsg_act.ActionStarted()) {
          wsg_act.OpenGripper();
        }

        if (wsg_act.ActionFinished()) {
          state = 7;
          wsg_act.Reset();
        }

      case 7:
        if (!iiwa_move.ActionStarted()) {
          std::vector<KukaIkPlanner::IkCartesianWaypoint> waypoints;
          KukaIkPlanner::IkCartesianWaypoint wp;
          wp.time = 0;
          wp.pose.translation() << 0, 0.68, z_low;
          wp.pose.linear() = Matrix3<double>(AngleAxis<double>(M_PI / 2., Vector3<double>::UnitZ()));
          wp.enforce_quat = true;

          for (int i = 0; i < N; i++) {
            wp.time += dt;
            wp.pose.translation()[2] += dz;
            waypoints.push_back(wp);
          }

          iiwa_move.MoveEnedEffectorThroughWaypoints(waypoints);
          std::cout << "exe 5 move: " << iiwa_move.get_iiwa_time() << std::endl;
        }

        if (iiwa_move.ActionFinished()) {
          state = 8;
          iiwa_move.Reset();
        }
        break;
    }
  }

  return 0;
}

/*
int main(int argc, const char* argv[]) {
  std::string path = GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf";
  KukaIkPlanner planner(path);

  KukaIkPlanner::IkResult ik_res;
  std::vector<KukaIkPlanner::IkCartesianWaypoint> waypoints;

  DRAKE_DEMAND(argc == 2);
  int move = atoi(argv[1]);

  KukaIkPlanner::IkCartesianWaypoint wp;

  int N = 3;
  double z_high = 0.3;
  double z_low = 0.1;
  double dz = (z_high - z_low) / N;
  double dt = 0.3;

  // approach
  wp.time = 1;
  wp.pose.translation() << 0.68, 0, z_high;
  wp.enforce_quat = true;

  switch (move) {
    case 0:
      waypoints.push_back(wp);
      break;
    case 1:
      // down
      wp.pose.translation()[2] = z_high;
      for (int i = 0; i < N; i++) {
        wp.time += dt;
        wp.pose.translation()[2] -= dz;
        waypoints.push_back(wp);
      }
      break;

    case 2:
      // up
      wp.pose.translation()[2] = z_low;
      for (int i = 0; i < N; i++) {
        wp.time += dt;
        wp.pose.translation()[2] += dz;
        waypoints.push_back(wp);
      }
      break;

    case 3:
      // approach other
      wp.time += 1;
      wp.pose.translation() << 0, 0.68, z_high;
      wp.pose.linear() = Matrix3<double>(AngleAxis<double>(M_PI / 2., Vector3<double>::UnitZ()));
      waypoints.push_back(wp);
      break;

    case 4:
      // down
      wp.pose.translation() << 0, 0.68, z_high;
      wp.pose.linear() = Matrix3<double>(AngleAxis<double>(M_PI / 2., Vector3<double>::UnitZ()));
      for (int i = 0; i < N; i++) {
        wp.time += dt;
        wp.pose.translation()[2] -= dz;
        waypoints.push_back(wp);
      }
      break;

    case 5:
      // up
      wp.pose.translation() << 0, 0.68, z_low;
      wp.pose.linear() = Matrix3<double>(AngleAxis<double>(M_PI / 2., Vector3<double>::UnitZ()));
      for (int i = 0; i < N; i++) {
        wp.time += dt;
        wp.pose.translation()[2] += dz;
        waypoints.push_back(wp);
      }
      break;
  }

  planner.PlanTrajectory(waypoints, VectorX<double>::Zero(7), &ik_res);

  robotlocomotion::robot_plan_t plan = planner.EncodeMessage(ik_res);
  lcm::LCM lcm;
  return lcm.publish(kLcmPlanChannel, &plan);
}
*/

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake


int main(int argc, const char* argv[]) {
  return drake::examples::kuka_iiwa_arm::main(argc, argv);
}
