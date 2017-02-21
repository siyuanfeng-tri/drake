#include <iostream>
#include <list>
#include <memory>

#include <lcm/lcm-cpp.hpp>

#include "bot_core/robot_state_t.hpp"
#include "drake/common/drake_path.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_ik_planner.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place_state.h"
#include "drake/lcmt_iiwa_status.hpp"

#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/util/lcmUtil.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place_demo {
namespace {

// Object dimension
const double kObjHalfHeight = 0.1;
const double kObjHeight = 2 * kObjHalfHeight;

// Desired location to place the object for table0 and table1. Positions are
// specified in the iiwa arm base frame.
const Vector3<double> kPlacePosition0(0.9, 0, 0);
const Vector3<double> kPlacePosition1(0, 0.9, 0);

// TODO(siyuan): have a better way to determine which table is the object on.
int get_table(const Isometry3<double>& X_WObj,
              const Isometry3<double>& X_WIiiwa) {
  Isometry3<double> X_IiwaObj = X_WIiiwa.inverse() * X_WObj;
  double dist_to_table_0 =
      (X_IiwaObj.translation() - kPlacePosition0).norm();
  double dist_to_table_1 =
      (X_IiwaObj.translation() - kPlacePosition1).norm();

  int table = 1;
  if (dist_to_table_0 < dist_to_table_1) table = 0;
  return table;
}

// Different states for the pick and place task.
enum PickAndPlaceState {
  OPEN_GRIPPER,
  APPROACH_PICK_PREGRASP,
  APPROACH_PICK,
  GRASP,
  LIFT_FROM_PICK,
  APPROACH_PLACE_PREGRASP,
  APPROACH_PLACE,
  PLACE,
  LIFT_FROM_PLACE,
  DONE,
};

// Makes a state machine that drives the iiwa to pick up a block from one table
// and place it on on the other.
int main(int argc, const char* argv[]) {
  lcm::LCM lcm;

  const int kNumLinearMotionSegments = 3;
  const double kLinearMotionDuration = 0.3;
  const double kEndEffectorToMidFingerDepth = 0.11;

  const std::string iiwa_path =
      GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf";
  const std::string iiwa_end_effector_name = "iiwa_link_ee";

  // Makes a EnvState, and sets up LCM subscriptions.
  EnvState env_state(iiwa_path, iiwa_end_effector_name, &lcm);
  env_state.SubscribeToWsgStatus("SCHUNK_WSG_STATUS");
  env_state.SubscribeToIiwaStatus("IIWA_STATE_EST");
  env_state.SubscribeToObjectStatus("OBJECT_STATE_EST");

  // Spins until we get at least 1 message from all channels.
  while (lcm.handleTimeout(10) == 0 ||
         env_state.get_iiwa_time() == -1 ||
         env_state.get_obj_time() == -1 ||
         env_state.get_wsg_time() == -1) {
  }

  // Makes a planner.
  const Isometry3<double> iiwa_base = env_state.get_iiwa_base();
  std::shared_ptr<RigidBodyFrame<double>> iiwa_base_frame =
      std::make_shared<RigidBodyFrame<double>>("world", nullptr, iiwa_base);
  IiwaIkPlanner planner(iiwa_path, iiwa_end_effector_name, iiwa_base);
  IiwaIkPlanner::IkResult ik_res;

  // Makes action handles.
  WsgAction wsg_act("SCHUNK_WSG_COMMAND", &lcm);
  IiwaMove iiwa_move(env_state.get_iiwa(), "COMMITTED_ROBOT_PLAN", &lcm);

  // Desired end effector pose in the world frame for pick and place.
  Isometry3<double> X_WEndEffector0, X_WEndEffector1;

  // Desired object end pose relative to the base of the iiwa arm.
  Isometry3<double> X_IiwaObj_desired;

  // Desired object end pose in the world frame.
  Isometry3<double> X_WObj_desired;

  // Set initial state
  PickAndPlaceState state = OPEN_GRIPPER;

  // lcm handle loop
  while (true) {
    // Handles all messages.
    while (lcm.handleTimeout(10) == 0) {
    }

    switch (state) {
      // Opens the gripper.
      case OPEN_GRIPPER:
        if (!wsg_act.ActionStarted()) {
          wsg_act.OpenGripper(env_state);
          std::cout << "OPEN_GRIPPER: " << env_state.get_iiwa_time()
                    << std::endl;
        }

        if (wsg_act.ActionFinished(env_state)) {
          state = APPROACH_PICK_PREGRASP;
          wsg_act.Reset();
        }
        break;

      // Uses 2 seconds to approach 20cm above the center of the object.
      case APPROACH_PICK_PREGRASP:
        if (!iiwa_move.ActionStarted()) {
          // Sets desired end effector location to be 11cm behind the object,
          // with the same orientation relative to the object frame.
          Isometry3<double> X_ObjEndEffector_desired;
          X_ObjEndEffector_desired.translation() = Vector3<double>(-kEndEffectorToMidFingerDepth, 0, 0);
          X_ObjEndEffector_desired.linear().setIdentity();

          // Desired gripper pose in the world frame.
          X_WEndEffector0 = env_state.get_object_pose() * X_ObjEndEffector_desired;
          std::cout << "X_WEndEffector0 location: "
                    << X_WEndEffector0.translation().transpose() << std::endl;
          std::cout << "X_WEndEffector0 rotation: " << X_WEndEffector0.linear()
                    << std::endl;

          std::vector<IiwaIkPlanner::IkCartesianWaypoint> waypoints(1);
          waypoints.front().time = 2;
          waypoints.front().pose = X_WEndEffector0;
          waypoints.front().pose.translation()[2] += kObjHeight;
          waypoints.front().enforce_quat = true;

          bool res = planner.PlanSequentialTrajectory(waypoints, env_state.get_iiwa_q(), &ik_res);
          DRAKE_DEMAND(res);
          iiwa_move.MoveJoints(env_state, ik_res.time, ik_res.q);

          std::cout << "APPROACH_PICK_PREGRASP: " << env_state.get_iiwa_time()
                    << std::endl;
        }

        if (iiwa_move.ActionFinished(env_state)) {
          state = APPROACH_PICK;
          iiwa_move.Reset();
        }
        break;

      // Moves gripper straight down.
      case APPROACH_PICK:
        if (!iiwa_move.ActionStarted()) {
          std::vector<IiwaIkPlanner::IkCartesianWaypoint> waypoints;
          IiwaIkPlanner::IkCartesianWaypoint wp;
          wp.time = 0;
          wp.pose = X_WEndEffector0;
          wp.pose.translation()[2] += kObjHeight;
          wp.enforce_quat = true;
          double dz = -kObjHeight / kNumLinearMotionSegments;

          for (int i = 0; i < kNumLinearMotionSegments; i++) {
            wp.time += kLinearMotionDuration;
            wp.pose.translation()[2] += dz;
            waypoints.push_back(wp);
          }

          bool res = planner.PlanSequentialTrajectory(waypoints, env_state.get_iiwa_q(), &ik_res);
          DRAKE_DEMAND(res);
          iiwa_move.MoveJoints(env_state, ik_res.time, ik_res.q);
          std::cout << "APPROACH_PICK: " << env_state.get_iiwa_time()
                    << std::endl;
        }

        if (iiwa_move.ActionFinished(env_state)) {
          state = GRASP;
          iiwa_move.Reset();
        }
        break;

      // Grasps the object.
      case GRASP:
        if (!wsg_act.ActionStarted()) {
          wsg_act.CloseGripper(env_state);
          std::cout << "GRASP: " << env_state.get_iiwa_time() << std::endl;
        }

        if (wsg_act.ActionFinished(env_state)) {
          state = LIFT_FROM_PICK;
          wsg_act.Reset();
        }
        break;

      // Lifts the object straight up.
      case LIFT_FROM_PICK:
        if (!iiwa_move.ActionStarted()) {
          std::vector<IiwaIkPlanner::IkCartesianWaypoint> waypoints;
          IiwaIkPlanner::IkCartesianWaypoint wp;
          wp.time = 0;
          wp.pose = X_WEndEffector0;
          wp.enforce_quat = true;
          double dz = kObjHeight / kNumLinearMotionSegments;

          for (int i = 0; i < kNumLinearMotionSegments; i++) {
            wp.time += kLinearMotionDuration;
            wp.pose.translation()[2] += dz;
            waypoints.push_back(wp);
          }


          bool res = planner.PlanSequentialTrajectory(waypoints, env_state.get_iiwa_q(), &ik_res);
          DRAKE_DEMAND(res);
          iiwa_move.MoveJoints(env_state, ik_res.time, ik_res.q);
          std::cout << "LIFT_FROM_PICK: " << env_state.get_iiwa_time()
                    << std::endl;
        }

        if (iiwa_move.ActionFinished(env_state)) {
          state = APPROACH_PLACE_PREGRASP;
          iiwa_move.Reset();
        }
        break;

      // Ueses 2 seconds to move to right about the X_WEndEffector1 location.
      case APPROACH_PLACE_PREGRASP:
        if (!iiwa_move.ActionStarted()) {
          int table = get_table(env_state.get_object_pose(), iiwa_base);

          // Sets desired place location based on where we picked up the object.
          // Table 0 is in front of iiwa base, and table 1 is to the left.
          if (table == 0) {
            // Table 0 -> table 1.
            X_IiwaObj_desired.translation() << 0, 0.9, 0.1;
            X_IiwaObj_desired.linear() = Matrix3<double>(
                AngleAxis<double>(M_PI / 2., Vector3<double>::UnitZ()));
          } else {
            // Table 1 -> table 0.
            X_IiwaObj_desired.translation() << 0.9, 0, 0.1;
            X_IiwaObj_desired.linear().setIdentity();
          }
          X_WObj_desired = iiwa_base * X_IiwaObj_desired;

          // Recomputes gripper's pose relative the object since the object
          // probably moved during transfer.
          const Isometry3<double> X_ObjEndEffector =
              env_state.get_object_pose().inverse() * env_state.get_iiwa_end_effector_pose();

          // Desired end effector pose in the world.
          X_WEndEffector1 = X_WObj_desired * X_ObjEndEffector;
          std::cout << "X_WEndEffector1 location: "
                    << X_WEndEffector1.translation().transpose() << std::endl;
          std::cout << "X_WEndEffector1 rotation: " << X_WEndEffector1.linear()
                    << std::endl;

          const double duration = 2;
          const int num_via_points = 2;
          const double dt = duration / (num_via_points + 1);
          std::vector<IiwaIkPlanner::IkCartesianWaypoint> waypoints;
          IiwaIkPlanner::IkCartesianWaypoint wp;

          // Makes a slerp trajectory from start to end.
          std::vector<double> times = {0, duration};
          const eigen_aligned_std_vector<Quaternion<double>> quats = {
              Quaternion<double>(X_WEndEffector0.linear()),
              Quaternion<double>(X_WEndEffector1.linear())};

          const std::vector<MatrixX<double>> pos = {
              X_WEndEffector0.translation(), X_WEndEffector1.translation()};
          PiecewiseQuaternionSlerp<double> rot_traj(times, quats);
          PiecewisePolynomial<double> pos_traj =
              PiecewisePolynomial<double>::FirstOrderHold(times, pos);

          for (int i = 1; i < num_via_points; ++i) {
            wp.time = dt * i;
            wp.pose.translation() =
                pos_traj.value(wp.time) + Vector3<double>(0, 0, 0.3);
            wp.pose.linear() = Matrix3<double>(rot_traj.orientation(wp.time));
            wp.pos_tol = Vector3<double>(0.02, 0.02, 0.02);
            wp.rot_tol = 0.5;
            wp.enforce_quat = true;
            waypoints.push_back(wp);
          }

          wp.time = duration;
          wp.pose = X_WEndEffector1;
          wp.pose.translation()[2] += kObjHeight;
          wp.enforce_quat = true;
          waypoints.push_back(wp);

          bool res = planner.PlanSequentialTrajectory(waypoints, env_state.get_iiwa_q(), &ik_res);
          DRAKE_DEMAND(res);
          iiwa_move.MoveJoints(env_state, ik_res.time, ik_res.q);
          std::cout << "APPROACH_PLACE_PREGRASP: " << env_state.get_iiwa_time()
                    << std::endl;
        }

        if (iiwa_move.ActionFinished(env_state)) {
          state = APPROACH_PLACE;
          iiwa_move.Reset();
        }
        break;

      // Moves straight down.
      case APPROACH_PLACE:
        if (!iiwa_move.ActionStarted()) {
          // Recomputes gripper's pose relative the object since the object
          // probably moved during transfer.
          const Isometry3<double> X_ObjEndEffector =
              env_state.get_object_pose().inverse() * env_state.get_iiwa_end_effector_pose();

          // Computes the desired end effector pose in the world frame.
          X_WEndEffector1 = X_WObj_desired * X_ObjEndEffector;

          std::vector<IiwaIkPlanner::IkCartesianWaypoint> waypoints;
          IiwaIkPlanner::IkCartesianWaypoint wp;
          wp.time = 0;
          wp.pose = X_WEndEffector1;
          // TODO(siyuan): This hack is to prevent the robot from forcefully
          // pushing the object into the table. Shouldn't be necessary once
          // we have guarded moves supported by the controller side.
          const double z_shift = 0.02;
          wp.pose.translation()[2] += kObjHeight + z_shift;
          wp.enforce_quat = true;

          double dz = -kObjHeight / kNumLinearMotionSegments;

          for (int i = 0; i < kNumLinearMotionSegments; i++) {
            wp.time += kLinearMotionDuration;
            wp.pose.translation()[2] += dz;
            std::cout << wp.pose.translation()[2] << std::endl;
            waypoints.push_back(wp);
          }

          bool res = planner.PlanSequentialTrajectory(waypoints, env_state.get_iiwa_q(), &ik_res);
          DRAKE_DEMAND(res);
          iiwa_move.MoveJoints(env_state, ik_res.time, ik_res.q);
          std::cout << "APPROACH_PLACE: " << env_state.get_iiwa_time()
                    << std::endl;
        }

        if (iiwa_move.ActionFinished(env_state)) {
          state = PLACE;
          iiwa_move.Reset();
        }
        break;

      // Releases the object.
      case PLACE:
        if (!wsg_act.ActionStarted()) {
          wsg_act.OpenGripper(env_state);
          std::cout << "PLACE: " << env_state.get_iiwa_time() << std::endl;
        }

        if (wsg_act.ActionFinished(env_state)) {
          state = LIFT_FROM_PLACE;
          wsg_act.Reset();
        }
        break;

      // Moves straight up.
      case LIFT_FROM_PLACE:
        if (!iiwa_move.ActionStarted()) {
          std::vector<IiwaIkPlanner::IkCartesianWaypoint> waypoints;
          IiwaIkPlanner::IkCartesianWaypoint wp;
          wp.time = 0;
          wp.pose = X_WEndEffector1;
          wp.enforce_quat = true;

          double dz = 2 * kObjHeight / kNumLinearMotionSegments;

          for (int i = 0; i < kNumLinearMotionSegments; i++) {
            wp.time += kLinearMotionDuration;
            wp.pose.translation()[2] += dz;
            waypoints.push_back(wp);
          }

          std::cout << "obj: "
                    << env_state.get_object_pose().translation().transpose()
                    << std::endl;
          std::cout << "obj: " << env_state.get_object_pose().linear()
                    << std::endl;

          bool res = planner.PlanSequentialTrajectory(waypoints, env_state.get_iiwa_q(), &ik_res);
          DRAKE_DEMAND(res);
          iiwa_move.MoveJoints(env_state, ik_res.time, ik_res.q);
          std::cout << "LIFT_FROM_PLACE: " << env_state.get_iiwa_time()
                    << std::endl;
        }

        if (iiwa_move.ActionFinished(env_state)) {
          state = OPEN_GRIPPER;
          iiwa_move.Reset();
        }
        break;

      case DONE:
        if (!iiwa_move.ActionStarted()) {
          const std::vector<double> time = {0, 2};
          MatrixX<double> q(iiwa_move.get_iiwa().get_num_positions(),
                            time.size());
          q.col(0) = env_state.get_iiwa_q();
          q.col(1).setZero();
          iiwa_move.MoveJoints(env_state, time, q);
          std::cout << "DONE: " << env_state.get_iiwa_time() << std::endl;
        }

        if (iiwa_move.ActionFinished(env_state)) {
          state = OPEN_GRIPPER;
          iiwa_move.Reset();
        }
        break;
    }
  }

  return 0;
}

}  // namespace
}  // namespace pick_and_place_demo
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, const char* argv[]) {
  return drake::examples::kuka_iiwa_arm::pick_and_place_demo::main(argc, argv);
}
