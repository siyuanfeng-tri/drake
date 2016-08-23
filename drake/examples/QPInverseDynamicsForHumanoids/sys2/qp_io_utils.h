#pragma once
#include "../qp_controller.h"
#include "../humanoid_status.h"

// robot state <-> flat vec
inline int get_humanoid_status_size(const HumanoidStatus &rs) {
  return rs.robot().number_of_positions() + rs.robot().number_of_velocities() + rs.robot().actuators.size() + 1 + 12;
}
void VectorXd2HumanoidStatus(const Eigen::VectorXd &v, HumanoidStatus *rs);
void HumanoidStatus2VectorXd(const HumanoidStatus &rs, Eigen::VectorXd *v);

// qp input <-> flat vec
inline int get_qp_input_size(const QPInput &input) {
  return 3 + 6 * 4 + input.vd_d().size() + 6 * 2 + 6;
}
void Vectorxd2QPInput(const Eigen::VectorXd &v, QPInput *input);
void QPInput2VectorXd(const QPInput &input, Eigen::VectorXd *v);

// qp output <-> flat vec
inline int get_qp_output_size(const QPOutput &output) {
  return 3 + 6 * 4 + output.vd().size() + output.joint_torque().size() + 12 + 12;
}
void VectorXd2QPOutput(const Eigen::VectorXd &v, QPOutput *output);
void QPOutput2VectorXd(const QPOutput &output, Eigen::VectorXd *v);

