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
#include "drake/examples/kuka_iiwa_arm/jjz_controller.h"

namespace drake {
namespace examples {
namespace jjz {

const std::string JjzController::kLcmStatusChannel = "IIWA_STATUS";
const std::string JjzController::kLcmCommandChannel = "IIWA_COMMAND";
const std::string JjzController::kLcmJjzControllerDebug = "CTRL_DEBUG";

void JjzController::MoveJ(const VectorX<double>& q_des, double duration) {
  PrimitiveOutput cur_output;
  GetPrimitiveOutput(&cur_output);

  std::unique_ptr<MotionPrimitive> new_plan(new MoveJoint("MoveJ", cur_output.q_cmd, q_des, 3.));
  SwapPlan(std::move(new_plan));
}

void JjzController::ControlLoop() {
  ////////////////////////////////////////////////////////
  // state related
  iiwa_status_.utime = -1;
  IiwaState state(&robot_, &frame_T_);

  // VERY IMPORTANT HACK, To make sure that no stale messages are in the
  // queue.
  for (int i = 0; i < 5; i++) {
    while (0 >= lcm_.handleTimeout(10) || iiwa_status_.utime == -1) {
    }
  }

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
  primitive_.reset(new MoveJoint("hold_q", state.get_q(), state.get_q(), 0.1));

  ////////////////////////////////////////////////////////
  // Main loop.
  while (run_flag_) {
    // Call lcm handle until at least one status message is
    // processed.
    while (0 >= lcm_.handleTimeout(10) || iiwa_status_.utime == -1) {
    }

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
}  // namespace examples
}  // namespace drake





