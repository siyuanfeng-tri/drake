#pragma once

#include <lcm/lcm-cpp.hpp>
#include "drake/examples/kuka_iiwa_arm/jjz_primitives.h"

namespace drake {
namespace examples {
namespace jjz {

class JjzController {
 public:
  static const std::string kLcmStatusChannel;
  static const std::string kLcmCommandChannel;
  static const std::string kLcmJjzControllerDebug;

  JjzController(const RigidBodyTree<double>& robot)
      : robot_(robot), frame_T_("tool", robot_.FindBody(kEEName), X_ET) {
    lcm::Subscription* sub =
        lcm_.subscribe(kLcmStatusChannel, &JjzController::HandleStatus, this);
    // THIS IS VERY IMPORTANT!!
    sub->setQueueCapacity(1);
  }

  const RigidBodyTree<double>& get_robot() const { return robot_; }
  const RigidBodyFrame<double>& get_tool_frame() const { return frame_T_; }

  // Starts the controller thread, needs to be called before GetState,
  // GetPrimitiveOutput,
  // or exectuing any motions.
  void Start() {
    if (run_flag_) {
      std::cout << "Controller thread already running\n";
      return;
    }
    run_flag_ = true;
    iiwa_status_.utime = -1;
    control_thread_ = std::thread(&JjzController::ControlLoop, this);
  }

  void Stop() {
    run_flag_ = false;
    control_thread_.join();
  }

  void GetState(IiwaState* state) const {
    DRAKE_DEMAND(run_flag_);

    lcmt_iiwa_status stats;
    while (true) {
      GetStateMsg(&stats);
      if (stats.utime == -1) {
        std::cout << "hasn't got a valid state yet.\n";
        usleep(1e4);
      } else {
        break;
      }
    }
    state->UpdateState(stats);
  }

  void GetPrimitiveOutput(PrimitiveOutput* output) const {
    DRAKE_DEMAND(run_flag_);

    std::lock_guard<std::mutex> guard(motion_lock_);
    *output = primitive_output_;
  }

  void MoveJ(const VectorX<double>& q_des, double duration);

  void MoveStraightUntilTouch(const Vector3<double>& dir_W, double vel,
                              double force_thresh);

  // Fz is in world frame.
  // If you don't want to deal with any of the force crap, just set
  // them all the zero. If you don't want to compensate for friction, set mus to
  // zero.
  //
  // Note: the timing in traj should be relative, i.e time range = [0, 2], not
  // absolute
  // clock.
  void MoveToolFollowTraj(
      const manipulation::PiecewiseCartesianTrajectory<double>& traj, double Fz,
      double mu, double yaw_mu);

 private:
  void SwapPlan(std::unique_ptr<MotionPrimitive> new_plan) {
    std::lock_guard<std::mutex> guard(motion_lock_);
    primitive_ = std::move(new_plan);
    primitive_output_.status = PrimitiveOutput::UNINIT;
  }

  void ControlLoop();

  void HandleStatus(const lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_iiwa_status* status) {
    SetStateMsg(*status);
  }

  void SetStateMsg(const lcmt_iiwa_status& msg) {
    std::lock_guard<std::mutex> guard(state_lock_);
    iiwa_status_ = msg;
  }

  void GetStateMsg(lcmt_iiwa_status* msg) const {
    std::lock_guard<std::mutex> guard(state_lock_);
    *msg = iiwa_status_;
  }

  lcm::LCM lcm_;
  const RigidBodyTree<double>& robot_;
  const RigidBodyFrame<double> frame_T_;

  mutable std::mutex state_lock_;
  lcmt_iiwa_status iiwa_status_{};

  mutable std::mutex motion_lock_;
  std::unique_ptr<MotionPrimitive> primitive_;
  PrimitiveOutput primitive_output_;

  std::thread control_thread_;
  std::atomic<bool> run_flag_{false};
};

}  // namespace jjz
}  // namespace examples
}  // namespace drake
