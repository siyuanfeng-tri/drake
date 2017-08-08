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
#include "drake/examples/kuka_iiwa_arm/jjz_controller.h"
#include "drake/examples/kuka_iiwa_arm/jjz_primitives.h"

namespace drake {
namespace jjz {

const std::string JjzController::kLcmStatusChannel = "IIWA_STATUS";
const std::string JjzController::kLcmCommandChannel = "IIWA_COMMAND";
const std::string JjzController::kLcmJjzControllerDebug = "CTRL_DEBUG";

void JjzController::MoveJ(const VectorX<double>& q_des, double duration) {
  PrimitiveOutput cur_output;
  GetPrimitiveOutput(&cur_output);

  std::unique_ptr<MotionPrimitive> new_plan(
      new MoveJoint("MoveJ", cur_output.q_cmd, q_des, duration));
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

void JjzController::ControlLoop() {
  // We can directly read iiwa_status_ because write only happens in
  // HandleStatus, which is only triggered by calling lcm_.handle,
  // which is only called from this func (thus, in the same thread).

  ////////////////////////////////////////////////////////
  // state related
  IiwaState state(&robot_, &frame_T_);

  // VERY IMPORTANT HACK, To make sure that no stale messages are in the
  // queue.
  int valid_msg_ctr = 0;
  int lcm_err;
  do {
    lcm_err = lcm_.handleTimeout(100);
    // > 0 got a message, = 0 timed out, < 0
    if (lcm_err > 0) {
      if (iiwa_status_.utime != -1) valid_msg_ctr++;
      if (valid_msg_ctr > 4) break;
    }
  } while (lcm_err <= 0);
  std::cout << "got first msg\n";

  ////////////////////////////////////////////////////////
  // cmd related
  lcmt_iiwa_command iiwa_command{};
  iiwa_command.num_joints = robot_.get_num_positions();
  iiwa_command.joint_position.resize(robot_.get_num_positions(), 0.);
  iiwa_command.num_torques = robot_.get_num_positions();
  iiwa_command.joint_torque.resize(robot_.get_num_positions(), 0.);

  VectorX<double> q_cmd(7);
  VectorX<double> trq_cmd = VectorX<double>::Zero(7);

  // Make initial plan.
  DRAKE_DEMAND(state.UpdateState(iiwa_status_));
  for (int i = 0; i < robot_.get_num_positions(); i++) {
    q_cmd[i] = iiwa_status_.joint_position_measured[i];
  }

  // Make initial plan to go to q1.
  auto plan = std::unique_ptr<MotionPrimitive>(
      new MoveJoint("hold_q", state.get_q(), state.get_q(), 0.1));
  SwapPlan(std::move(plan));

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
      }
    } while (lcm_err <= 0);

    // Update state.
    DRAKE_DEMAND(state.UpdateState(iiwa_status_));
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
    }

    // send command
    iiwa_command.utime = static_cast<int64_t>(state.get_time() * 1e6);
    for (int i = 0; i < robot_.get_num_positions(); i++) {
      iiwa_command.joint_position[i] = q_cmd[i];
      iiwa_command.joint_torque[i] = trq_cmd[i];
    }
    lcm_.publish(kLcmCommandChannel, &iiwa_command);

    // send debug msg.
    lcm_.publish(kLcmJjzControllerDebug, &ctrl_debug);
  }
}

}  // namespace jjz
}  // namespace drake
