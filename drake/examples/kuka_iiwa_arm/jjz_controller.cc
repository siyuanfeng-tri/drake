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
#include "drake/lcmt_schunk_wsg_command.hpp"
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
#include "drake/examples/kuka_iiwa_arm/jjz_controller.h"
#include "drake/examples/kuka_iiwa_arm/jjz_primitives.h"

#include "drake/lcmt_viewer_draw.hpp"

namespace drake {
namespace jjz {

const std::string JjzController::kLcmIiwaStatusChannel = "IIWA_STATUS";
const std::string JjzController::kLcmIiwaCommandChannel = "IIWA_COMMAND";
const std::string JjzController::kLcmJjzControllerDebug = "CTRL_DEBUG";
const std::string JjzController::kLcmWsgStatusChannel = "SCHUNK_WSG_STATUS";
const std::string JjzController::kLcmWsgCommandChannel = "SCHUNK_WSG_COMMAND";

JjzController::JjzController(const RigidBodyTree<double>& robot,
                             const RigidBodyFrame<double>& frame_T)
    : robot_(robot), frame_T_(frame_T) {
  // Iiwa status.
  lcm::Subscription* sub = lcm_.subscribe(
      kLcmIiwaStatusChannel, &JjzController::HandleIiwaStatus, this);
  // THIS IS VERY IMPORTANT!!
  sub->setQueueCapacity(1);

  // Gripper status.
  sub = lcm_.subscribe(kLcmWsgStatusChannel, &JjzController::HandleWsgStatus,
                       this);
  sub->setQueueCapacity(1);
}

void JjzController::Start() {
  if (run_flag_) {
    std::cout << "Controller thread already running\n";
    return;
  }
  run_flag_ = true;
  ready_flag_ = false;
  iiwa_msg_ctr_ = 0;
  wsg_msg_ctr_ = 0;

  control_thread_ = std::thread(&JjzController::ControlLoop, this);

  // Block until ready.
  while (!ready_flag_)
    ;
}

void JjzController::Stop() {
  if (!run_flag_) {
    std::cout << "Controller thread not running\n";
    return;
  }

  run_flag_ = false;
  ready_flag_ = false;
  control_thread_.join();
}

void JjzController::GetPrimitiveOutput(PrimitiveOutput* output) const {
  DRAKE_DEMAND(run_flag_ && ready_flag_);

  std::lock_guard<std::mutex> guard(motion_lock_);
  *output = primitive_output_;
}

void JjzController::HandleIiwaStatus(const lcm::ReceiveBuffer*,
                                     const std::string&,
                                     const lcmt_iiwa_status* status) {
  std::lock_guard<std::mutex> guard(state_lock_);
  iiwa_status_ = *status;
  iiwa_msg_ctr_++;
}

void JjzController::HandleWsgStatus(const lcm::ReceiveBuffer*,
                                    const std::string&,
                                    const lcmt_schunk_wsg_status* status) {
  std::lock_guard<std::mutex> guard(state_lock_);
  wsg_status_ = *status;
  wsg_msg_ctr_++;
}

void JjzController::MoveJ(const VectorX<double>& q_des, double duration) {
  PrimitiveOutput cur_output;
  GetPrimitiveOutput(&cur_output);

  std::unique_ptr<MotionPrimitive> new_plan(
      new MoveJoint("MoveJ", &get_robot(), cur_output.q_cmd, q_des, duration));
  SwapPlan(std::move(new_plan));
}

void JjzController::MoveJ(const std::vector<VectorX<double>>& q_des,
                          const std::vector<double>& duration) {
  PrimitiveOutput cur_output;
  GetPrimitiveOutput(&cur_output);

  std::unique_ptr<MotionPrimitive> new_plan(
      new MoveJoint("MoveJ", &get_robot(), cur_output.q_cmd, q_des, duration));
  SwapPlan(std::move(new_plan));
}

void JjzController::MoveJ(const VectorX<double>& q_des) {
  PrimitiveOutput cur_output;
  GetPrimitiveOutput(&cur_output);

  std::unique_ptr<MotionPrimitive> new_plan(
      new MoveJoint("MoveJ", &get_robot(), cur_output.q_cmd, q_des));
  SwapPlan(std::move(new_plan));
}

void JjzController::MoveStraightUntilTouch(const Vector3<double>& dir_W,
                                           double vel, double force_thresh) {
  PrimitiveOutput cur_output;
  GetPrimitiveOutput(&cur_output);

  auto new_plan = new MoveToolStraightUntilTouch(
      "MoveStraightUntilTouch", &get_robot(), &get_tool_frame(),
      cur_output.q_cmd, dir_W, vel);
  new_plan->set_f_ext_thresh(force_thresh);
  SwapPlan(std::unique_ptr<MotionPrimitive>(new_plan));
}

void JjzController::MoveToolFollowTraj(
    const manipulation::PiecewiseCartesianTrajectory<double>& traj, double Fz,
    double mu, double yaw_mu) {
  PrimitiveOutput cur_output;
  GetPrimitiveOutput(&cur_output);

  auto new_plan =
      new class MoveToolFollowTraj("MoveToolFollowTraj", &get_robot(),
                                   &get_tool_frame(), cur_output.q_cmd, traj);
  new_plan->set_fz(Fz);
  new_plan->set_mu(mu);
  new_plan->set_yaw_mu(yaw_mu);
  SwapPlan(std::unique_ptr<MotionPrimitive>(new_plan));
}

void JjzController::GetIiwaState(IiwaState* state) const {
  DRAKE_DEMAND(run_flag_);

  lcmt_iiwa_status stats;
  while (true) {
    std::lock_guard<std::mutex> guard(state_lock_);
    if (iiwa_msg_ctr_ == 0) {
      std::cout << "hasn't got a valid iiwa state yet.\n";
    } else {
      stats = iiwa_status_;
      break;
    }
  }
  state->UpdateState(stats);
}

void JjzController::GetWsgState(WsgState* state) const {
  DRAKE_DEMAND(run_flag_);

  lcmt_schunk_wsg_status stats;
  while (true) {
    std::lock_guard<std::mutex> guard(state_lock_);
    if (wsg_msg_ctr_ == 0) {
      std::cout << "hasn't got a valid wsg state yet.\n";
    } else {
      stats = wsg_status_;
      break;
    }
  }
  state->UpdateState(stats);
}

void JjzController::SetGripperPositionAndForce(double position, double force) {
  lcmt_schunk_wsg_command cmd{};
  // TODO set timestamp.
  cmd.target_position_mm = position;
  cmd.force = force;
  lcm_.publish(kLcmWsgCommandChannel, &cmd);
}

void FillFrameMessage(const Isometry3<double>& pose, int idx,
    drake::lcmt_viewer_draw* msg) {
  for (int j = 0; j < 3; j++)
      msg->position[idx][j] = static_cast<float>(pose.translation()[j]);

  Quaternion<double> quat(pose.linear());
  msg->quaternion[idx][0] = static_cast<float>(quat.w());
  msg->quaternion[idx][1] = static_cast<float>(quat.x());
  msg->quaternion[idx][2] = static_cast<float>(quat.y());
  msg->quaternion[idx][3] = static_cast<float>(quat.z());
}

void JjzController::ControlLoop() {
  // We can directly read iiwa_status_ because write only happens in
  // HandleIiwaStatus, which is only triggered by calling lcm_.handle,
  // which is only called from this func (thus, in the same thread).

  ////////////////////////////////////////////////////////
  // state related
  IiwaState state(&robot_, &frame_T_);

  // VERY IMPORTANT HACK, To make sure that no stale messages are in the
  // queue.
  int lcm_err;
  while (iiwa_msg_ctr_ < 4) {
    lcm_err = lcm_.handleTimeout(10);
  }
  std::cout << "got first iiwa msg\n";

  ////////////////////////////////////////////////////////
  // cmd related
  lcmt_iiwa_command iiwa_command{};
  iiwa_command.num_joints = robot_.get_num_positions();
  iiwa_command.joint_position.resize(robot_.get_num_positions(), 0.);
  iiwa_command.num_torques = robot_.get_num_positions();
  iiwa_command.joint_torque.resize(robot_.get_num_positions(), 0.);

  VectorX<double> q_cmd(7);
  VectorX<double> trq_cmd = VectorX<double>::Zero(7);
  Isometry3<double> X_WT_cmd = Isometry3<double>::Identity();

  // Make initial plan.
  DRAKE_DEMAND(state.UpdateState(iiwa_status_));
  for (int i = 0; i < robot_.get_num_positions(); i++) {
    q_cmd[i] = iiwa_status_.joint_position_measured[i];
  }

  // Make initial plan to go to q1.
  auto plan = std::unique_ptr<MotionPrimitive>(
      new MoveJoint("hold_q", &robot_, state.get_q(), state.get_q(), 0.1));
  SwapPlan(std::move(plan));
  {
    lcmt_jjz_controller ctrl_debug{};
    std::lock_guard<std::mutex> guard(motion_lock_);
    primitive_->Initialize(state);
    primitive_->Update(state, &ctrl_debug);
    primitive_->Control(state, &primitive_output_, &ctrl_debug);
  }
  ready_flag_ = true;

  // Frame visualziation stuff.
  Isometry3<double> GRASP = Isometry3<double>::Identity();

  Isometry3<double> tf_camera_wrt_ee;
  tf_camera_wrt_ee.matrix() <<
    0.3994,   -0.9168,   -0.0015,   -0.0646,
    0.9163,    0.3992,   -0.0317,    0.00111,
    0.0297,    0.0113,    0.9995,    0.1121,
         0,         0,         0,    1.0000;
  Isometry3<double> tf_hand_to_ee(
    Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, 0.185)) *
    Eigen::AngleAxis<double>(-22. / 180. * M_PI, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxis<double>(M_PI, Eigen::Vector3d::UnitY()));

  Isometry3<double> tf_camera_wrt_hand = tf_hand_to_ee.inverse() * tf_camera_wrt_ee;

  drake::lcmt_viewer_draw frame_msg{};
  frame_msg.link_name = {"Tool_measured", "Tool_ik", "Grasp"};
  frame_msg.num_links = frame_msg.link_name.size();
  // The robot num is not relevant here.
  frame_msg.robot_num.resize(frame_msg.num_links, 0);
  std::vector<float> pos = {0, 0, 0};
  std::vector<float> quaternion = {1, 0, 0, 0};
  frame_msg.position.resize(frame_msg.num_links, pos);
  frame_msg.quaternion.resize(frame_msg.num_links, quaternion);

  ////////////////////////////////////////////////////////
  // Main loop.
  while (run_flag_) {
    // Call lcm handle until at least one status message is
    // processed.
    do {
      lcm_err = lcm_.handleTimeout(10);
      if (lcm_err == 0) {
        std::cout << "LCM recv timed out in control loop 10ms.\n";
      } else if (lcm_err < 0) {
        std::cout << "LCM recv error.\n";
      } else {
        // Update state.
        if (state.UpdateState(iiwa_status_)) break;
      }
    } while (lcm_err <= 0);

    lcmt_jjz_controller ctrl_debug{};
    FillDebugMessage(state, &ctrl_debug);

    // Locked by motion_lock_
    {
      std::lock_guard<std::mutex> guard(motion_lock_);
      if (!primitive_->is_init()) {
        primitive_->Initialize(state);
      }
      primitive_->Update(state, &ctrl_debug);
      primitive_->Control(state, &primitive_output_, &ctrl_debug);

      q_cmd = primitive_output_.q_cmd;
      trq_cmd = primitive_output_.trq_cmd;
      X_WT_cmd = primitive_output_.X_WT_cmd;

      GRASP = primitive_->HACK_COMP_GRASP_POSE();
    }

    // Generate frame visualization stuff.
    frame_msg.timestamp = ctrl_debug.utime;
    FillFrameMessage(state.get_X_WT() * tf_camera_wrt_hand.inverse(), 0, &frame_msg);
    FillFrameMessage(X_WT_cmd * tf_camera_wrt_hand.inverse(), 1, &frame_msg);
    FillFrameMessage(GRASP * tf_camera_wrt_hand.inverse(), 2, &frame_msg);
    lcm_.publish("DRAKE_DRAW_FRAMES", &frame_msg);

    // send command
    iiwa_command.utime = static_cast<int64_t>(state.get_time() * 1e6);
    for (int i = 0; i < robot_.get_num_positions(); i++) {
      iiwa_command.joint_position[i] = q_cmd[i];
      iiwa_command.joint_torque[i] = trq_cmd[i];
    }
    lcm_.publish(kLcmIiwaCommandChannel, &iiwa_command);

    // send debug msg.
    lcm_.publish(kLcmJjzControllerDebug, &ctrl_debug);
  }
}

}  // namespace jjz
}  // namespace drake
