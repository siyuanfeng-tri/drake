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

    iiwa_base_ = Isometry3<double>::Identity();
    iiwa_q_ = VectorX<double>::Zero(7);
    iiwa_v_ = VectorX<double>::Zero(7);
    obj_pose_ = Isometry3<double>::Identity();
    wsg_q_ = 0;
    wsg_force_ = 0;
  }

  virtual void HandleIiwaStatus(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const bot_core::robot_state_t* iiwa_msg) {
    iiwa_time_ = iiwa_msg->utime / 1e6;

    iiwa_base_ = DecodePose(iiwa_msg->pose);

    for (int i = 0; i < iiwa_msg->num_joints; i++) {
      iiwa_v_[i] = iiwa_msg->joint_velocity[i];
      iiwa_q_[i] = iiwa_msg->joint_position[i];
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

  double ComputeObjectLowestZ() const {
    Isometry3<double> center_to_bottom = Isometry3<double>::Identity();
    const Vector3<double> offsets[4] = {Vector3<double>(0.03, 0.03, -0.1),
                                        Vector3<double>(0.03, -0.03, -0.1),
                                        Vector3<double>(-0.03, 0.03, -0.1),
                                        Vector3<double>(-0.03, -0.03, -0.1)};
    double min_z = 1100;
    for (int i = 0; i < 4; i++) {
      center_to_bottom.translation() = offsets[i];
      double z = (get_object_pose() * center_to_bottom).translation()[2];
      if (z < min_z)
        min_z = z;
    }
    return min_z;
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

  const Isometry3<double>& get_iiwa_base() const { return iiwa_base_; }

  const Vector6<double>& get_object_velocity() const { return obj_vel_; }

  virtual void Reset() {
    act_start_time_ = -1;
  }

 protected:
  double act_start_time_;
  double iiwa_time_;

  Isometry3<double> iiwa_base_;
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
  IiwaMoveCartesian(lcm::LCM* lcm, const std::string& channel, const std::string& iiwa_model_path, std::shared_ptr<RigidBodyFrame<double>> iiwa_base)
      : planner_(iiwa_model_path, "iiwa_link_ee", iiwa_base), iiwa_(planner_.get_robot()), cache_(iiwa_.CreateKinematicsCache()), lcm_(lcm), channel_(channel) {
    end_effector_ = iiwa_.FindBody("iiwa_link_ee");
  }

  void HandleIiwaStatus(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const bot_core::robot_state_t* iiwa_msg) {
    Action::HandleIiwaStatus(rbuf, chan, iiwa_msg);
    cache_.initialize(iiwa_q_, iiwa_v_);
    iiwa_.doKinematics(cache_, true);
    mid_finger_pose_ = iiwa_.CalcBodyPoseInWorldFrame(cache_, *end_effector_);
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

  const Isometry3<double>& get_mid_finger_pose() const { return mid_finger_pose_; }

  // TODO
  bool ActionFailed() const {
    return false;
  }

  bool ActionFinished() const {
    if (!ActionStarted())
      return false;

    if (iiwa_time_ - act_start_time_ > waypoints_.back().time && iiwa_v_.norm() < 3e-2) {
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

  const RigidBody<double>* end_effector_;
  Isometry3<double> mid_finger_pose_;
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

    if (std::abs(wsg_v_) < 1e-2 && (wsg_time_ - act_start_time_ > 0.5))
      return true;
    return false;
  }

 private:
  lcm::LCM* lcm_;
  const std::string channel_;
};

int main(int argc, const char* argv[]) {
  lcm::LCM lcm;

  WsgAction wsg_act(&lcm, "SCHUNK_WSG_COMMAND");
  lcm.subscribe("IIWA_STATE_EST", &Action::HandleIiwaStatus, (Action*)&wsg_act);
  // get the base pose of iiwa
  while (0 == lcm.handleTimeout(10) || wsg_act.get_iiwa_time() == -1) {}
  Isometry3<double> iiwa_base = wsg_act.get_iiwa_base();
  std::shared_ptr<RigidBodyFrame<double>> iiwa_base_frame =
      std::make_shared<RigidBodyFrame<double>>("world", nullptr, iiwa_base);

  std::string iiwa_path = GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf";
  IiwaMoveCartesian iiwa_move(&lcm, kLcmPlanChannel, iiwa_path, iiwa_base_frame);

  lcm.subscribe("IIWA_STATE_EST", &IiwaMoveCartesian::HandleIiwaStatus, &iiwa_move);
  lcm.subscribe("OBJECT_STATE_EST", &Action::HandleObjectStatus, (Action*)&iiwa_move);
  lcm.subscribe("SCHUNK_WSG_STATUS", &Action::HandleWsgStatus, (Action*)&wsg_act);
  lcm.subscribe("OBJECT_STATE_EST", &Action::HandleObjectStatus, (Action*)&wsg_act);

  int state = -1;

  int N = 3;
  double dt = 0.3;

  Isometry3<double> pick, place;
  pick.translation() = iiwa_base.translation() + Vector3<double>(0.68, 0, 0.1); // iiwa_move.get_object_pose().translation();
  pick.linear() = Matrix3<double>::Identity();
  place.translation() = iiwa_base.translation() + Vector3<double>(0, 0.68, 0.1);
  place.linear() = Matrix3<double>(AngleAxis<double>(M_PI / 2., Vector3<double>::UnitZ()));

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
        break;

      case 0:
        if (!iiwa_move.ActionStarted()) {
          std::vector<KukaIkPlanner::IkCartesianWaypoint> waypoints;
          KukaIkPlanner::IkCartesianWaypoint wp;
          wp.time = 2;
          wp.pose = pick;
          wp.pose.translation()[2] += 0.2;
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
          wp.pose = pick;
          wp.pose.translation()[2] += 0.2;
          wp.enforce_quat = true;
          double dz = -0.2 / N;

          for (int i = 0; i < N; i++) {
            wp.time += dt;
            wp.pose.translation()[2] += dz;
            waypoints.push_back(wp);
          }

          iiwa_move.MoveEnedEffectorThroughWaypoints(waypoints);
          std::cout << "exe 1 move: " << iiwa_move.get_iiwa_time() << std::endl;
        }

        if (iiwa_move.ActionFinished()) {
          state = 2;
          iiwa_move.Reset();
          std::cout << "obj z: " << iiwa_move.get_object_pose().translation()[2];
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
          wp.pose = pick;
          wp.enforce_quat = true;
          double dz = 0.2 / N;

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

          wp.time = 1;
          wp.pose.translation() = (pick.translation() + place.translation()) / 2.;
          wp.pose.translation()[2] += 0.5;
          wp.enforce_quat = false;
          waypoints.push_back(wp);

          wp.time = 2;
          wp.pose = place;
          wp.pose.translation()[2] += 0.2;
          wp.enforce_quat = true;
          waypoints.push_back(wp);

          iiwa_move.MoveEnedEffectorThroughWaypoints(waypoints);
          std::cout << "exe 3 move: " << iiwa_move.get_iiwa_time() << std::endl;
        }

        if (iiwa_move.ActionFinished() &&
            iiwa_move.get_object_velocity().head<2>().norm() < 5e-2) {
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
          wp.pose = place;
          wp.pose.translation()[2] += 0.2;
          wp.enforce_quat = true;

          std::cout << "gripper: " << iiwa_move.get_mid_finger_pose().translation().transpose() << std::endl;
          std::cout << "obj: " << iiwa_move.get_object_pose().translation().transpose() << std::endl;

          double table_z = iiwa_base.translation()[2];
          double min_obj_z = iiwa_move.ComputeObjectLowestZ();

          double z_delta = table_z - min_obj_z + 0.01;
          double dz = z_delta / N;

          for (int i = 0; i < N; i++) {
            wp.time += dt;
            wp.pose.translation()[2] += dz;
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
        break;

      case 7:
        if (!iiwa_move.ActionStarted()) {
          std::vector<KukaIkPlanner::IkCartesianWaypoint> waypoints;
          KukaIkPlanner::IkCartesianWaypoint wp;
          wp.time = 0;
          wp.pose = place;
          wp.enforce_quat = true;

          double dz = 0.2 / N;

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

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake


int main(int argc, const char* argv[]) {
  return drake::examples::kuka_iiwa_arm::main(argc, argv);
}
