/// @file
///
/// Generates a canned IK demo plan for an iiwa arm starting from the
/// zero configuration and sends that plan over lcm using the
/// robot_plan_t message.

#include "drake/examples/kuka_iiwa_arm/kuka_ik_planner.h"

#include <list>
#include <iostream>
#include <memory>

#include <lcm/lcm-cpp.hpp>


#include "drake/multibody/constraint/rigid_body_constraint.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_ik.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

KukaIkPlanner::KukaIkPlanner(const std::string& model_path, std::shared_ptr<RigidBodyFrame<double>> base) {
  robot_ = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFile(model_path,
      multibody::joints::kFixed, base, robot_.get());

  end_effector_body_idx_ = robot_->FindBodyIndex("iiwa_link_ee");
}

bool KukaIkPlanner::PlanTrajectory(const std::vector<IkCartesianWaypoint>& waypoints, const VectorX<double>& q_current, IkResult* ik_res) {
  int num_dof = robot_->get_num_positions();
  int num_steps = static_cast<int>(waypoints.size());
  MatrixX<double> q0(num_dof, num_steps);

  for (int i = 0; i < num_steps; i++) {
    const auto& waypoint = waypoints[i];
    // this could be better
    q0.col(i) = q_current;

    // TODO: THIS IS A BIG HACK
    q0(0, i) = atan2(waypoint.pose.translation()[1], waypoint.pose.translation()[0]);
  }

  Vector3<double> pos_tol(1, 1, 1);
  double ang_tol = 0.5;
  bool ret;

  while (pos_tol.norm() > 0.001 || ang_tol > 0.01) {
    ret = PlanTrajectory(waypoints, q_current, q0, ik_res, pos_tol, ang_tol);
    // success, reduce pos
    if (ret) {
      q0 = ik_res->q.block(0, 1, num_dof, num_steps);
      pos_tol *= 0.5;
      ang_tol *= 0.5;
    } else {
      pos_tol *= 1.5;
      ang_tol *= 1.5;
      std::cout << "failed at: " << pos_tol.transpose() << ", " << ang_tol << std::endl;
    }
  }

  return true;
}

bool KukaIkPlanner::PlanTrajectory(const std::vector<IkCartesianWaypoint>& waypoints, const VectorX<double>& q_current, const MatrixX<double>& q0, IkResult* ik_res, const Vector3<double>& position_tol, double rot_tolerance) {
  DRAKE_DEMAND(ik_res);
  int num_dof = robot_->get_num_positions();
  int num_steps = static_cast<int>(waypoints.size());
  MatrixX<double> q_nom(q0), q_sol(q0);
  std::vector<double> times(num_steps);
  std::vector<int> info(num_steps);
  std::vector<std::string> infeasible_constraint;
  std::vector<RigidBodyConstraint*> constraint_array;
  IKoptions ikoptions(robot_.get());
  ikoptions.setDebug(true);

  std::list<std::unique_ptr<RigidBodyConstraint>> constraints;

  // start from q_current
  VectorX<double> joint_lb = q_current - VectorX<double>::Constant(7, 0.01);
  VectorX<double> joint_ub = q_current + VectorX<double>::Constant(7, 0.01);
  PostureConstraint pc1(robot_.get(), Vector2<double>(0, 0.1));
  VectorX<int> joint_idx(num_dof);
  for (int i = 0; i < num_dof; i++)
    joint_idx[i] = i;
  pc1.setJointLimits(joint_idx, joint_lb, joint_ub);
  constraint_array.push_back(&pc1);

  for (int i = 0; i < num_steps; i++) {
    const auto& waypoint = waypoints[i];
    times[i] = waypoint.time;

    // make position constraint
    Vector3<double> pos_lb = waypoint.pose.translation() - position_tol;
    Vector3<double> pos_ub = waypoint.pose.translation() + position_tol;
    // need to make sure time doesnt overlap
    Vector2<double> tspan(waypoint.time - 0.1, waypoint.time + 0.1);

    // pos constraints
    std::unique_ptr<WorldPositionConstraint> pos_con =
        std::make_unique<WorldPositionConstraint>(
            robot_.get(), end_effector_body_idx_, Vector3<double>::Zero(), pos_lb, pos_ub,
            tspan);
    constraints.push_back(std::move(pos_con));

    constraint_array.push_back(constraints.back().get());

    // rot constraints
    if (waypoints[i].enforce_quat) {
      std::unique_ptr<WorldQuatConstraint> quat_con =
          std::make_unique<WorldQuatConstraint>(robot_.get(), end_effector_body_idx_,
                                                math::rotmat2quat(waypoint.pose.linear()), rot_tolerance, tspan);
      constraints.push_back(std::move(quat_con));
      constraint_array.push_back(constraints.back().get());
    }
  }

  std::cout << "num constraints: " << constraint_array.size() << ", num wp: " << num_steps << std::endl;

  // this could be better
  q_nom.setZero();

  inverseKinPointwise(robot_.get(), num_steps, times.data(), q0, q_nom,
                      constraint_array.size(), constraint_array.data(),
                      ikoptions, &q_sol, info.data(), &infeasible_constraint);

  bool info_good = true;
  for (int i = 0; i < num_steps; ++i) {
    printf("INFO[%d] = %d ", i, info[i]);
    if (info[i] != 1) {
      info_good = false;
    }
  }
  printf("\n");

  for (const auto& s : infeasible_constraint) {
    std::cout << "haha: " << s << std::endl;
  }

  ik_res->time.resize(num_steps + 1);
  ik_res->info.resize(num_steps + 1);
  ik_res->q.resize(num_dof, num_steps + 1);

  ik_res->time[0] = 0;
  ik_res->info[0] = 1;
  ik_res->q.col(0) = q_current;
  for (int i = 0; i < num_steps; i++) {
    ik_res->time[i+1] = times[i];
    ik_res->info[i+1] = info[i];
    ik_res->q.col(i+1) = q_sol.col(i);
  }

  if (!info_good) {
    std::cerr << "Solution failed, not sending." << std::endl;
    return false;
  }

  return true;
}

robotlocomotion::robot_plan_t KukaIkPlanner::EncodeMessage(const IkResult& ik_res) {
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
  plan.utime = 0;  // I (sam.creasey) don't think this is used?
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
