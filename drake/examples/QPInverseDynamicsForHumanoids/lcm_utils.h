#pragma once

#include <string>
#include <vector>
#include <unordered_map>

#include <Eigen/Dense>

#include "bot_core/robot_state_t.hpp"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"

#include "drake/lcmt_constrained_values.hpp"
#include "drake/lcmt_contact_information.hpp"
#include "drake/lcmt_desired_centroidal_momentum_dot.hpp"
#include "drake/lcmt_desired_dof_motions.hpp"
#include "drake/lcmt_desired_body_motion.hpp"
#include "drake/lcmt_qp_input.hpp"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

ConstraintType DecodeConstraintType(int8_t type);
int8_t EncodeConstraintType(ConstraintType type);

void DecodeQPInput(const RigidBodyTree& robot, const lcmt_qp_input& msg, QPInput* qp_input);
void EncodeQPInput(const QPInput& qp_input, lcmt_qp_input* msg);

void DecodeConstrainedValues(const lcmt_constrained_values& msg, ConstrainedValues* val);
void EncodeConstrainedValues(const ConstrainedValues& val, lcmt_constrained_values* msg);

void DecodeContactInformation(const RigidBodyTree& robot, const lcmt_contact_information& msg, ContactInformation* info);
void EncodeContactInformation(const ContactInformation& info, lcmt_contact_information* msg);

void DecodeDesiredCentroidalMomentumDot(const lcmt_desired_centroidal_momentum_dot& msg, DesiredCentroidalMomentumDot* momdot);
void EncodeDesiredCentroidalMomentumDot(const DesiredCentroidalMomentumDot& momdot, lcmt_desired_centroidal_momentum_dot* msg);

void DecodeDesiredDoFMotions(const lcmt_desired_dof_motions& msg, DesiredDoFMotions* dof_motions);
void EncodeDesiredDoFMotions(const DesiredDoFMotions& dof_motions, lcmt_desired_dof_motions* msg);

void DecodeDesiredBodyMotion(const RigidBodyTree& robot, const lcmt_desired_body_motion& msg, DesiredBodyMotion* body_motion);
void EncodeDesiredBodyMotion(const DesiredBodyMotion& body_motion, lcmt_desired_body_motion* msg);

// TODO(siyuan.feng) Replace these with Twan's similar Encode / Decode methods.
/**
 * Make a bot_core::robot_state_t lcm message based on given information.
 * The current implementation assumes:
 * 1. \p q and \p qd have the same dimension.
 * 2. Has a RPY floating base joint.
 * 3. All DOF are actuated except the floating base.
 *
 * @param act_joint_names List of ACTUATED joint names (excluding floating base)
 * @param time Time
 * @param q generalized position (including floating base)
 * @param qd generalized velocity (including floating base)
 * @param joint_torque Actuated joint torques
 * @param l_foot_wrench Left foot wrench in the sensor frame
 * @param r_foot_wrench Right foot wrench in the sensor frame
 * @param msg Output lcm message
 */
void EncodeRobotStateLcmMsg(const std::vector<std::string>& act_joint_names,
                            double time, const Eigen::VectorXd& q,
                            const Eigen::VectorXd& qd,
                            const Eigen::VectorXd& joint_torque,
                            const Vector6<double>& l_foot_wrench,
                            const Vector6<double>& r_foot_wrench,
                            bot_core::robot_state_t* msg);

/**
 * Decode state related information from a bot_core::robot_state_t message.
 * \p msg can contain extra joint information, but we will only decode
 * joints that are listed in \p q_name_to_index.
 * The current implementation assumes:
 * 1. \p q and \p qd have the same dimension.
 * 2. Has a RPY floating base joint.
 * 3. All DOF are actuated except the floating base.
 *
 * @param msg Lcm message
 * @param q_name_to_index A map from coordinate names to index, the joint_name
 * in \p msg is used for lookup.
 * @param time Pointer to decoded time
 * @param q Pointer to decoded generalized position
 * @param qd Pointer to decoded generalized velocity
 * @param joint_torque Pointer to decoded generalized joint torque
 * @param l_foot_wrench Pointer to decoded left foot wrench in the sensor frame
 * @param r_foot_wrench Pointer to decoded right foot wrench in the sensor frame
 */
void DecodeRobotStateLcmMsg(
    const bot_core::robot_state_t& msg,
    const std::unordered_map<std::string, int>& q_name_to_index, double* time,
    Eigen::VectorXd* q, Eigen::VectorXd* qd, Eigen::VectorXd* joint_torque,
    Vector6<double>* l_foot_wrench, Vector6<double>* r_foot_wrench);

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
