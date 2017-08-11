#pragma once

#include <lcm/lcm-cpp.hpp>
#include "drake/examples/kuka_iiwa_arm/jjz_primitives.h"

namespace drake {
namespace jjz {

class JjzController {
 public:
  static const std::string kLcmIiwaStatusChannel;
  static const std::string kLcmIiwaCommandChannel;
  static const std::string kLcmJjzControllerDebug;
  static const std::string kLcmWsgStatusChannel;
  static const std::string kLcmWsgCommandChannel;

  JjzController(const RigidBodyTree<double>& robot,
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

  const RigidBodyTree<double>& get_robot() const { return robot_; }
  const RigidBodyFrame<double>& get_tool_frame() const { return frame_T_; }

  // Starts the controller thread, needs to be called before GetIiwaState,
  // GetPrimitiveOutput,
  // or exectuing any motions.
  void Start() {
    if (run_flag_) {
      std::cout << "Controller thread already running\n";
      return;
    }
    run_flag_ = true;
    iiwa_msg_ctr_ = 0;
    wsg_msg_ctr_ = 0;

    control_thread_ = std::thread(&JjzController::ControlLoop, this);
  }

  void Stop() {
    run_flag_ = false;
    control_thread_.join();
  }

  // This blocks until there is at least 1 valid status message from iiwa.
  void GetIiwaState(IiwaState* state) const;

  // This blocks until there is at least 1 valid status message from iiwa.
  void GetWsgState(WsgState* state) const;

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

  void CloseGripperAndSleep(double sec = 0) {
    std::cout << "[Close gripper]\n";
    SetGripperPositionAndForce(0, 40);
    if (sec <= 0) return;
    usleep(sec * 1e6);
  }

  void OpenGripperAndSleep(double sec = 0) {
    std::cout << "[Open gripper]\n";
    SetGripperPositionAndForce(105, 40);
    if (sec <= 0) return;
    usleep(sec * 1e6);
  }

  void SetGripperPositionAndForce(double position, double force);

 private:
  void SwapPlan(std::unique_ptr<MotionPrimitive> new_plan) {
    std::lock_guard<std::mutex> guard(motion_lock_);
    primitive_ = std::move(new_plan);
    primitive_output_.status = PrimitiveOutput::UNINIT;
  }

  void ControlLoop();

  void HandleIiwaStatus(const lcm::ReceiveBuffer*, const std::string&,
                        const lcmt_iiwa_status* status) {
    std::lock_guard<std::mutex> guard(state_lock_);
    iiwa_status_ = *status;
    iiwa_msg_ctr_++;
  }

  void HandleWsgStatus(const lcm::ReceiveBuffer*, const std::string&,
                       const lcmt_schunk_wsg_status* status) {
    std::lock_guard<std::mutex> guard(state_lock_);
    wsg_status_ = *status;
    wsg_msg_ctr_++;
  }

  lcm::LCM lcm_;
  const RigidBodyTree<double>& robot_;
  const RigidBodyFrame<double> frame_T_;

  mutable std::mutex state_lock_;
  lcmt_iiwa_status iiwa_status_{};
  lcmt_schunk_wsg_status wsg_status_{};
  int iiwa_msg_ctr_{0};
  int wsg_msg_ctr_{0};

  mutable std::mutex motion_lock_;
  std::unique_ptr<MotionPrimitive> primitive_;
  PrimitiveOutput primitive_output_;

  std::thread control_thread_;
  std::atomic<bool> run_flag_{false};
};

}  // namespace jjz
}  // namespace drake
