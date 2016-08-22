#include "qp_io_utils.h"
#include "drake/common/drake_assert.h"

void VectorXd2HumanoidStatus(const Eigen::VectorXd &v, HumanoidStatus *rs) {
  if (v.size() != get_humanoid_status_size(*rs)) {
    DRAKE_ASSERT(false);
    return;
  }

  int time_idx = 0;
  int q_idx = time_idx + 1;
  int v_idx = q_idx + rs->robot().number_of_positions();
  int trq_idx = v_idx + rs->robot().number_of_velocities();
  int l_ft_idx = trq_idx + rs->robot().actuators.size();
  int r_ft_idx = l_ft_idx + 6;
  DRAKE_ASSERT(r_ft_idx + 6 == get_humanoid_status_size(*rs));

  rs->Update(v[time_idx],
      v.segment(q_idx, rs->robot().number_of_positions()),
      v.segment(v_idx, rs->robot().number_of_velocities()),
      v.segment(trq_idx, rs->robot().actuators.size()),
      v.segment<6>(l_ft_idx),
      v.segment<6>(r_ft_idx));
}

void HumanoidStatus2VectorXd(const HumanoidStatus &rs, Eigen::VectorXd *v) {
  if (v->size() != get_humanoid_status_size(rs)) {
    DRAKE_ASSERT(false);
    return;
  }

  int idx = 0;
  (*v)(idx) = rs.time(); idx += 1;
  v->segment(idx, rs.position().size()) = rs.position(); idx += rs.position().size();
  v->segment(idx, rs.velocity().size()) = rs.velocity(); idx += rs.velocity().size();
  v->segment(idx, rs.joint_torque().size()) = rs.joint_torque(); idx += rs.joint_torque().size();
  v->segment(idx, rs.foot_wrench_in_sensor_frame(Side::LEFT).size()) = rs.foot_wrench_in_sensor_frame(Side::LEFT); idx += rs.foot_wrench_in_sensor_frame(Side::LEFT).size();
  v->segment(idx, rs.foot_wrench_in_sensor_frame(Side::RIGHT).size()) = rs.foot_wrench_in_sensor_frame(Side::RIGHT); idx += rs.foot_wrench_in_sensor_frame(Side::RIGHT).size();

  DRAKE_ASSERT(idx == get_humanoid_status_size(rs));
}

void Vectorxd2QPInput(const Eigen::VectorXd &v, QPInput *input) {
  if (v.size() != get_qp_input_size(*input)) {
    DRAKE_ASSERT(false);
    return;
  }

  int idx = 0;
  input->comdd_d() = v.segment<3>(idx); idx += 3;
  input->pelvdd_d() = v.segment<6>(idx); idx += 6;
  input->torsodd_d() = v.segment<6>(idx); idx += 6;
  input->footdd_d(0) = v.segment<6>(idx); idx += 6;
  input->footdd_d(1) = v.segment<6>(idx); idx += 6;
  input->vd_d() = v.segment(idx, input->vd_d().size()); idx += input->vd_d().size();
  input->wrench_d(0) = v.segment<6>(idx); idx += 6;
  input->wrench_d(1) = v.segment<6>(idx); idx += 6;

  input->w_com = v(idx); idx += 1;
  input->w_pelv = v(idx); idx += 1;
  input->w_torso = v(idx); idx += 1;
  input->w_foot = v(idx); idx += 1;
  input->w_vd = v(idx); idx += 1;
  input->w_wrench_reg = v(idx); idx += 1;
  DRAKE_ASSERT(idx == get_qp_input_size(*input));
}

void QPInput2VectorXd(const QPInput &input, Eigen::VectorXd *v) {
  if (v->size() != get_qp_input_size(input)) {
    std::cout << get_qp_input_size(input) << " " << v->size() << std::endl;
    DRAKE_ASSERT(false);
    return;
  }

  int idx = 0;
  v->segment<3>(idx) = input.comdd_d(); idx += 3;
  v->segment<6>(idx) = input.pelvdd_d(); idx += 6;
  v->segment<6>(idx) = input.torsodd_d(); idx += 6;
  v->segment<6>(idx) = input.footdd_d(0); idx += 6;
  v->segment<6>(idx) = input.footdd_d(1); idx += 6;
  v->segment(idx, input.vd_d().size()) = input.vd_d(); idx += input.vd_d().size();
  v->segment<6>(idx) = input.wrench_d(0); idx += 6;
  v->segment<6>(idx) = input.wrench_d(1); idx += 6;

  (*v)(idx) = input.w_com; idx += 1;
  (*v)(idx) = input.w_pelv; idx += 1;
  (*v)(idx) = input.w_torso; idx += 1;
  (*v)(idx) = input.w_foot; idx += 1;
  (*v)(idx) = input.w_vd; idx += 1;
  (*v)(idx) = input.w_wrench_reg; idx += 1;

  DRAKE_ASSERT(idx == get_qp_input_size(input));
}

void VectorXd2QPOutput(const Eigen::VectorXd &v, QPOutput *output) {
  if (v.size() != get_qp_output_size(*output)) {
    DRAKE_ASSERT(false);
    return;
  }

  int idx = 0;
  output->comdd() = v.segment<3>(idx); idx += 3;
  output->pelvdd() = v.segment<6>(idx); idx += 6;
  output->torsodd() = v.segment<6>(idx); idx += 6;
  output->footdd(0) = v.segment<6>(idx); idx += 6;
  output->footdd(1) = v.segment<6>(idx); idx += 6;
  output->vd() = v.segment(idx, output->vd().size()); idx += output->vd().size();
  output->joint_torque() = v.segment(idx, output->joint_torque().size()); idx += output->joint_torque().size();
  output->foot_wrench_in_world_frame(0) = v.segment<6>(idx); idx += 6;
  output->foot_wrench_in_world_frame(1) = v.segment<6>(idx); idx += 6;
  output->foot_wrench_in_sensor_frame(0) = v.segment<6>(idx); idx += 6;
  output->foot_wrench_in_sensor_frame(1) = v.segment<6>(idx); idx += 6;

  DRAKE_ASSERT(idx == get_qp_output_size(*output));
}

void QPOutput2VectorXd(const QPOutput &output, Eigen::VectorXd *v) {
  if (v->size() != get_qp_output_size(output)) {
    DRAKE_ASSERT(false);
    return;
  }

  int idx = 0;
  v->segment<3>(idx) = output.comdd(); idx += 3;
  v->segment<6>(idx) = output.pelvdd(); idx += 6;
  v->segment<6>(idx) = output.torsodd(); idx += 6;
  v->segment<6>(idx) = output.footdd(0); idx += 6;
  v->segment<6>(idx) = output.footdd(1); idx += 6;
  v->segment(idx, output.vd().size()) = output.vd(); idx += output.vd().size();
  v->segment(idx, output.joint_torque().size()) = output.joint_torque(); idx += output.joint_torque().size();
  v->segment<6>(idx) = output.foot_wrench_in_world_frame(0); idx += 6;
  v->segment<6>(idx) = output.foot_wrench_in_world_frame(1); idx += 6;
  v->segment<6>(idx) = output.foot_wrench_in_sensor_frame(0); idx += 6;
  v->segment<6>(idx) = output.foot_wrench_in_sensor_frame(1); idx += 6;

  DRAKE_ASSERT(idx == get_qp_output_size(output));
}

