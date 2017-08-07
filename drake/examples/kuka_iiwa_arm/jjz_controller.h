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
      : robot_(robot),
        frame_T_("tool", robot_.FindBody(kEEName), X_ET) {
    lcm::Subscription* sub =
        lcm_.subscribe(kLcmStatusChannel, &JjzController::HandleStatus, this);
    // THIS IS VERY IMPORTANT!!
    sub->setQueueCapacity(1);
  }

  const RigidBodyTree<double>& get_robot() const { return robot_; }
  const RigidBodyFrame<double>& get_tool_frame() const { return frame_T_; }

  void Start() {
    if (run_flag_) {
      std::cout << "Controller thread already running\n";
      return;
    }
    run_flag_ = true;
    control_thread_ = std::thread(&JjzController::ControlLoop, this);
  }

  void Stop() {
    run_flag_ = false;
    control_thread_.join();
  }

  void GetState(IiwaState* state) const {
    DRAKE_DEMAND(run_flag_);
    lcmt_iiwa_status stats;
    while(true) {
      stats = CopyStateMsg();
      if (stats.utime == -1) {
        std::cout << "hasn't got a valid state yet.\n";
        usleep(1e5);
      } else {
        break;
      }
    }
    state->UpdateState(stats);
  }

  void MoveJ(const VectorX<double>& q_des, double duration);

  void GetPrimitiveOutput(PrimitiveOutput* output) const {
    std::lock_guard<std::mutex> guard(motion_lock_);
    *output = primitive_output_;
  }

 private:
  void SwapPlan(std::unique_ptr<MotionPrimitive> new_plan) {
    std::lock_guard<std::mutex> guard(motion_lock_);
    primitive_ = std::move(new_plan);
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

  lcmt_iiwa_status CopyStateMsg() const {
    std::lock_guard<std::mutex> guard(state_lock_);
    return iiwa_status_;
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
