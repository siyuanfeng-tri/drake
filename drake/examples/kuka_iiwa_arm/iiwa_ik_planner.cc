#include "drake/examples/kuka_iiwa_arm/iiwa_ik_planner.h"

#include <iostream>
#include <list>
#include <memory>

#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/multibody/constraint/rigid_body_constraint.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_ik.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

IiwaIkPlanner::IiwaIkPlanner(const std::string& model_path,
                             const std::string& end_effector_link_name,
                             const Isometry3<double>& base_to_world) {
  auto base_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "world", nullptr,
      base_to_world);

  robot_ = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFile(
      model_path, multibody::joints::kFixed, base_frame, robot_.get());

  SetEndEffector(end_effector_link_name);
}

IiwaIkPlanner::IiwaIkPlanner(const std::string& model_path,
                             const std::string& end_effector_link_name,
                             std::shared_ptr<RigidBodyFrame<double>> base) {
  robot_ = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFile(
      model_path, multibody::joints::kFixed, base, robot_.get());

  SetEndEffector(end_effector_link_name);

  KinematicsCache<double> cache = robot_->CreateKinematicsCache();
  cache.initialize(VectorX<double>::Zero(7), VectorX<double>::Zero(7));
  robot_->doKinematics(cache);
  std::cout << robot_->CalcBodyPoseInWorldFrame(cache, *robot_->FindBody(end_effector_link_name)).translation() << std::endl;
}

std::unique_ptr<PiecewisePolynomialTrajectory>
IiwaIkPlanner::GenerateFirstOrderHoldTrajectoryFromCartesianWaypoints(
    const std::vector<double>& time_stamps,
    const std::vector<Vector3<double>>& way_point_list,
    const Vector3<double>& position_tol, double rotation_tol) {
  DRAKE_DEMAND(way_point_list.size() == time_stamps.size());
  std::vector<IkCartesianWaypoint> waypoints(time_stamps.size());
  for (int i = 0; i < static_cast<int>(time_stamps.size()); ++i) {
    waypoints[i].time = time_stamps[i];
    waypoints[i].pose.translation() = way_point_list[i];
  }

  IkResult ik_res;
  PlanTrajectory(waypoints, robot_->getZeroConfiguration(), position_tol,
                 rotation_tol, &ik_res);
  return GenerateFirstOrderHoldTrajectory(ik_res);
}

bool IiwaIkPlanner::PlanTrajectory(
    const std::vector<IkCartesianWaypoint>& waypoints,
    const VectorX<double>& q_current, const Vector3<double>& position_tol,
    double rotation_tol, IkResult* ik_res) {
  std::cout << "stff\n";
  DRAKE_DEMAND(ik_res);
  int num_dof = robot_->get_num_positions();
  int num_steps = static_cast<int>(waypoints.size());

  std::cout << "num_dof " << num_dof << "num_steps " << num_steps << std::endl;

  VectorX<double> q_start = q_current;
  VectorX<double> q_end = q_current;

  ik_res->time.resize(num_steps + 1);
  ik_res->info.resize(num_steps + 1);
  ik_res->q.resize(num_dof, num_steps + 1);
  ik_res->time[0] = 0;
  ik_res->info[0] = 1;
  ik_res->q.col(0) = q_current;

  int ctr = 0;
  for (const auto& waypoint : waypoints) {
    Vector3<double> pos_tol = position_tol;
    double rot_tol = rotation_tol;

    // reduce pos
    int mode = 0;
    int itr = 0;

    // Solve point IK with constraint fiddling.
    while (true) {
      bool res = SolveIk(waypoint, q_start, pos_tol, rot_tol, &q_end);
      itr++;

      if (res) {
        if (mode == 0) {
          rot_tol /= 2.;
          mode = 1;
        } else {
          pos_tol /= 2.;
          mode = 0;
        }
      } else {
        if (mode == 0) {
          rot_tol *= 1.5;
        } else {
          pos_tol *= 1.5;
        }
      }

      if (pos_tol.norm() < 0.005 && rot_tol < 0.05) break;

      if (itr > 100) {
        std::cout << "FAILED AT MAX ITR\n";
        return false;
      }
    }

    // Set next IK's initial and bias to current solution.
    q_start = q_end;

    ik_res->time[ctr + 1] = waypoint.time;
    ik_res->info[ctr + 1] = 1;

    ik_res->q.col(ctr + 1) = q_end;
    ctr++;
  }

  return true;
}

bool IiwaIkPlanner::SolveIk(const IkCartesianWaypoint& waypoint,
                            const VectorX<double>& q_current,
                            const Vector3<double>& pos_tol, double rot_tol,
                            VectorX<double>* q_res) {
  DRAKE_DEMAND(q_res);
  std::vector<RigidBodyConstraint*> constraint_array;
  std::vector<int> info(1);
  std::vector<std::string> infeasible_constraint;
  IKoptions ikoptions(robot_.get());
  ikoptions.setDebug(true);

  // make position constraint
  Vector3<double> pos_lb = waypoint.pose.translation() - pos_tol;
  Vector3<double> pos_ub = waypoint.pose.translation() + pos_tol;

  WorldPositionConstraint pos_con(robot_.get(), end_effector_body_idx_,
                                  Vector3<double>::Zero(), pos_lb, pos_ub,
                                  Vector2<double>::Zero());

  constraint_array.push_back(&pos_con);

  // rot constraints
  WorldQuatConstraint quat_con(robot_.get(), end_effector_body_idx_,
                               math::rotmat2quat(waypoint.pose.linear()),
                               rot_tol, Vector2<double>::Zero());
  if (waypoint.enforce_quat) {
    constraint_array.push_back(&quat_con);
  }

  VectorX<double> q0 = q_current;
  VectorX<double> q_sol = q0;

  inverseKin(robot_.get(), q0, q_current, constraint_array.size(),
             constraint_array.data(), ikoptions, &q_sol, info.data(),
             &infeasible_constraint);

  *q_res = q_sol;

  if (info[0] != 1) {
    return false;
  }

  return true;
}

robotlocomotion::robot_plan_t IiwaIkPlanner::EncodeMessage(
    const IkResult& ik_res) {
  DRAKE_DEMAND(ik_res.q.cols() == static_cast<int>(ik_res.time.size()));
  DRAKE_DEMAND(ik_res.q.rows() == robot_->get_num_positions());
  DRAKE_DEMAND(ik_res.info.size() == ik_res.time.size());

  // Default robot_state for init.
  bot_core::robot_state_t default_robot_state;
  default_robot_state.num_joints = robot_->get_num_positions();
  default_robot_state.joint_name.resize(default_robot_state.num_joints);
  for (int i = 0; i < default_robot_state.num_joints; i++) {
    default_robot_state.joint_name[i] = robot_->get_position_name(i);
  }
  default_robot_state.joint_position.resize(default_robot_state.num_joints, 0);
  default_robot_state.joint_velocity.resize(default_robot_state.num_joints, 0);
  default_robot_state.joint_effort.resize(default_robot_state.num_joints, 0);

  robotlocomotion::robot_plan_t plan;
  plan.utime = 0;            // I (sam.creasey) don't think this is used?
  plan.robot_name = "iiwa";  // Arbitrary, probably ignored
  plan.num_states = ik_res.q.cols();
  plan.plan.resize(plan.num_states, default_robot_state);
  plan.plan_info.resize(plan.num_states, 0);

  for (int t = 0; t < plan.num_states; t++) {
    bot_core::robot_state_t& step = plan.plan[t];
    step.utime = ik_res.time[t] * 1e6;
    for (int j = 0; j < step.num_joints; j++) {
      step.joint_position[j] = ik_res.q(j, t);
    }
    plan.plan_info[t] = ik_res.info[t];
  }
  plan.num_grasp_transitions = 0;
  plan.left_arm_control_type = plan.POSITION;
  plan.right_arm_control_type = plan.NONE;
  plan.left_leg_control_type = plan.NONE;
  plan.right_leg_control_type = plan.NONE;
  plan.num_bytes = 0;

  return plan;
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
