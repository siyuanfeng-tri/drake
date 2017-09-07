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
                const RigidBodyFrame<double>& frame_T);

  const RigidBodyTree<double>& get_robot() const { return robot_; }
  const RigidBodyFrame<double>& get_tool_frame() const { return frame_T_; }

  // Starts the controller thread, needs to be called before GetIiwaState,
  // GetPrimitiveOutput, or exectuing any motions.
  void Start();
  void Stop();

  // This blocks until there is at least 1 valid status message from iiwa.
  void GetIiwaState(IiwaState* state) const;

  // This blocks until there is at least 1 valid status message from iiwa.
  void GetWsgState(WsgState* state) const;

  void GetPrimitiveOutput(PrimitiveOutput* output) const;

  void MoveJ(const VectorX<double>& q_des, double duration);
  void MoveJ(const std::vector<VectorX<double>>& q_des,
             const std::vector<double>& duration);

  void MoveJ(const VectorX<double>& q_des);

  void MoveStraightUntilTouch(const Vector3<double>& dir_W, double vel,
                              double force_thresh);

  // Fz is in world frame.
  // If you don't want to deal with any of the force crap, just set
  // them all the zero. If you don't want to compensate for friction, set mus to
  // zero.
  //
  // Note: the timing in traj should be relative, i.e time range = [0, 2], not
  // absolute clock.
  void MoveToolFollowTraj(
      const manipulation::PiecewiseCartesianTrajectory<double>& traj, double Fz,
      double mu, double yaw_mu);

  void CloseGripperAndSleep(double sec = 0);
  void OpenGripperAndSleep(double sec = 0);
  void SetGripperPositionAndForce(double position, double force);

 private:
  inline void SwapPlan(std::unique_ptr<MotionPrimitive> new_plan) {
    std::lock_guard<std::mutex> guard(motion_lock_);
    primitive_ = std::move(new_plan);
    primitive_output_.status = PrimitiveOutput::UNINIT;
  }

  void ControlLoop();

  void HandleIiwaStatus(const lcm::ReceiveBuffer*, const std::string&,
                        const lcmt_iiwa_status* status);
  void HandleWsgStatus(const lcm::ReceiveBuffer*, const std::string&,
                       const lcmt_schunk_wsg_status* status);

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
