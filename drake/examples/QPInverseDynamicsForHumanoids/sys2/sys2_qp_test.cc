#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/translator_between_lcmt_drake_signal.h"
#include "drake/lcmt_drake_signal.hpp"

#include "drake/systems/lcm/lcm_receive_thread.h"
#include "sys2_qp.h"
#include "drake/common/drake_path.h"

#include "../dummy_integrate.h"


class MessagePublisher {
 public:
  MessagePublisher(const std::string& channel_name, ::lcm::LCM* lcm, int size)
      : channel_name_(channel_name), lcm_(lcm) {

    message_.dim = size;
    message_.val.resize(message_.dim);
    message_.coord.resize(message_.dim);
    for (int ii = 0; ii < message_.dim; ++ii) {
      message_.val[ii] = NAN;
      message_.coord[ii] = "coord_" + std::to_string(ii);
    }
    message_.timestamp = 0;
  }

  void Start() {
    thread_.reset(new std::thread(&MessagePublisher::DoPublish, this));
  }

  void Stop() {
    stop_ = true;
    thread_->join();
  }

  void SetValue(const Eigen::VectorXd &v) {
    lock_.lock();
    if (v.size() != message_.dim) {
      lock_.unlock();
      return;
    }
    for (int i = 0; i < message_.dim; i++)
      message_.val[i] = v[i];
    lock_.unlock();
  }

 private:
  void DoPublish() {
    drake::lcmt_drake_signal msg;
    while (!stop_) {
      lock_.lock();
      msg = message_;
      lock_.unlock();

      lcm_->publish(channel_name_, &msg);
      //std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  }

  const std::string channel_name_;

  ::lcm::LCM* lcm_;

  std::mutex lock_;

  drake::lcmt_drake_signal message_;

  std::atomic<bool> stop_{false};

  std::unique_ptr<std::thread> thread_;
};

void testIO() {
  std::string urdf =
    drake::GetDrakePath() +
    std::string(
        "/examples/QPInverseDynamicsForHumanoids/valkyrie_sim_drake.urdf");

  HumanoidStatus rs(std::make_unique<RigidBodyTree>(urdf, DrakeJoint::ROLLPITCHYAW));
  QPInput qp_input;
  QPOutput qp_output;
  InitQPInput(rs.robot(), &qp_input);
  InitQPOutput(rs.robot(), &qp_output);

  Eigen::VectorXd X, X1, in, in1, out, out1;
  X = Eigen::VectorXd::Random(get_humanoid_status_size(rs));
  X1 = Eigen::VectorXd::Random(get_humanoid_status_size(rs));
  in = Eigen::VectorXd::Random(get_qp_input_size(qp_input));
  in1 = Eigen::VectorXd::Random(get_qp_input_size(qp_input));
  out = Eigen::VectorXd::Random(get_qp_output_size(qp_output));
  out1 = Eigen::VectorXd::Random(get_qp_output_size(qp_output));

  VectorXd2HumanoidStatus(X, &rs);
  HumanoidStatus2VectorXd(rs, &X1);
  DRAKE_ASSERT((X - X1).norm() == 0);

  Vectorxd2QPInput(in, &qp_input);
  QPInput2VectorXd(qp_input, &in1);
  DRAKE_ASSERT((in - in1).norm() == 0);

  VectorXd2QPOutput(out, &qp_output);
  QPOutput2VectorXd(qp_output, &out1);
  DRAKE_ASSERT((out - out1).norm() == 0);
}








int main() {
  testIO();

  lcm::LCM lcm;

  ////////////////////////////////////////////////////////////////
  // build the diagram
  std::string urdf =
    drake::GetDrakePath() +
    std::string(
        "/examples/QPInverseDynamicsForHumanoids/valkyrie_sim_drake.urdf");
  drake::systems::System2QP qp_con(urdf);
  qp_con.set_name("qp_controller");

  // 2 input
  drake::systems::lcm::TranslatorBetweenLcmtDrakeSignal state_trans(qp_con.number_of_robot_states());
  drake::systems::lcm::TranslatorBetweenLcmtDrakeSignal qp_input_trans(qp_con.number_of_qp_inputs());
  drake::systems::lcm::LcmSubscriberSystem state_sub("state", state_trans, &lcm);
  drake::systems::lcm::LcmSubscriberSystem qp_input_sub("qp_input", qp_input_trans, &lcm);
  state_sub.set_name("lcm_to_state");
  qp_input_sub.set_name("lcm_to_qp_input");
  // 1 output
  drake::systems::lcm::TranslatorBetweenLcmtDrakeSignal qp_output_trans(qp_con.number_of_qp_outputs());
  drake::systems::lcm::LcmPublisherSystem qp_output_pub("qp_output", qp_output_trans, &lcm);
  qp_output_pub.set_name("qp_output_to_lcm");

  // connect the diagram
  drake::systems::Diagram<double> mydiagram;
  mydiagram.set_name("Sys2 QP diagram");

  mydiagram.Connect(&state_sub, 0, &qp_con, 0);
  mydiagram.Connect(&qp_input_sub, 0, &qp_con, 1);
  mydiagram.Connect(&qp_con, 0, &qp_output_pub, 0);

  mydiagram.ExportOutput(&qp_con, 0);

  mydiagram.Finalize();
  ////////////////////////////////////////////////////////////////
  // lcm pub
  MessagePublisher state_pub("state", &lcm, qp_con.number_of_robot_states());
  state_pub.Start();
  MessagePublisher qp_input_pub("qp_input", &lcm, qp_con.number_of_qp_inputs());
  qp_input_pub.Start();
  std::cout << "pubs started\n";

  // lcm receiver
  drake::systems::lcm::LcmReceiveThread lcm_receive_thread(&lcm);

  ////////////////////////////////////////////////////////////////
  // fake simulation
  std::unique_ptr<drake::systems::ContextBase<double>> context = mydiagram.CreateDefaultContext();
  std::unique_ptr<drake::systems::SystemOutput<double>> output = mydiagram.AllocateOutput(*context);
  DRAKE_ASSERT(context->get_num_input_ports() == 0);
  DRAKE_ASSERT(output->get_num_ports() == 1);


  HumanoidStatus rs(std::make_unique<RigidBodyTree>(urdf, DrakeJoint::ROLLPITCHYAW));
  QPInput qp_input;
  QPOutput qp_output;
  InitQPInput(rs.robot(), &qp_input);
  InitQPOutput(rs.robot(), &qp_output);

  // initial condition
  Eigen::VectorXd STATE = Eigen::VectorXd::Zero(qp_con.number_of_robot_states());
  Eigen::VectorXd QP_INPUT = Eigen::VectorXd::Zero(qp_con.number_of_qp_inputs());
  Eigen::VectorXd QP_OUTPUT = Eigen::VectorXd::Zero(qp_con.number_of_qp_outputs());

  int time_idx = 0;
  int q_idx = time_idx + 1;
  int v_idx = q_idx + rs.robot().number_of_positions();
  int trq_idx = v_idx + rs.robot().number_of_velocities();
  int l_ft_idx = trq_idx + rs.robot().actuators.size();
  int r_ft_idx = l_ft_idx + 6;

  Eigen::Map<Eigen::VectorXd> q(STATE.data()+q_idx, rs.robot().number_of_positions());
  Eigen::Map<Eigen::VectorXd> qd(STATE.data()+v_idx, rs.robot().number_of_velocities());
  Eigen::Map<Eigen::VectorXd> trq(STATE.data()+trq_idx, rs.robot().actuators.size());
  Eigen::Map<Eigen::VectorXd> l_ft(STATE.data()+l_ft_idx, 6);
  Eigen::Map<Eigen::VectorXd> r_ft(STATE.data()+r_ft_idx, 6);

  q = rs.GetNominalPosition();
  VectorXd2HumanoidStatus(STATE, &rs);

  Vector6d Kp_default = (Vector6d() << 30, 30, 30, 30, 30, 30).finished();
  Vector6d Kd_default = (Vector6d() << 4, 4, 4, 4, 4, 4).finished();
  CartesianSetPoint pelv_fb(0, rs.pelv().pose, Vector6d::Zero(), Vector6d::Zero(), Kp_default, Kd_default);
  CartesianSetPoint torso_fb(0, rs.torso().pose, Vector6d::Zero(), Vector6d::Zero(), Kp_default, Kd_default);
  CartesianSetPoint foot_fb[2] = {CartesianSetPoint(0, rs.foot(Side::LEFT).pose, Vector6d::Zero(), Vector6d::Zero(), Kp_default, Kd_default),
                                  CartesianSetPoint(0, rs.foot(Side::RIGHT).pose, Vector6d::Zero(), Vector6d::Zero(), Kp_default, Kd_default)};

  Eigen::Vector3d com_d(rs.com());
  com_d[0] += 0.05;

  qp_input.pelvdd_d.setZero();
  qp_input.torsodd_d.setZero();
  qp_input.footdd_d[Side::LEFT].setZero();
  qp_input.footdd_d[Side::RIGHT].setZero();
  qp_input.wrench_d[Side::LEFT].setZero();
  qp_input.wrench_d[Side::RIGHT].setZero();
  qp_input.vd_d.setZero();
  // [5] is Fz, 660N * 2 is about robot weight.
  qp_input.wrench_d[Side::LEFT][5] = 660;
  qp_input.wrench_d[Side::RIGHT][5] = 660;
  qp_input.w_com = 1e2;
  qp_input.w_pelv = 1e1;
  qp_input.w_torso = 1e1;
  qp_input.w_foot = 1e1;
  qp_input.w_vd = 1e-1;
  qp_input.w_wrench_reg = 1e-6;

  VectorXd joint_d = rs.position().segment(6, rs.position().size()-6);
  int nj = joint_d.size();

  double dt = 0.01;
  while (true) {
    // make qp input
    qp_input.comdd_d = 40 * (com_d - rs.com()) - 4*rs.comd();
    qp_input.pelvdd_d = pelv_fb.ComputeAccelerationTarget(rs.pelv().pose, rs.pelv().vel);
    qp_input.torsodd_d = torso_fb.ComputeAccelerationTarget(rs.torso().pose, rs.torso().vel);
    qp_input.footdd_d[Side::LEFT] = foot_fb[Side::LEFT].ComputeAccelerationTarget(rs.foot(Side::LEFT).pose, rs.foot(Side::LEFT).vel);
    qp_input.footdd_d[Side::RIGHT] = foot_fb[Side::RIGHT].ComputeAccelerationTarget(rs.foot(Side::RIGHT).pose, rs.foot(Side::RIGHT).vel);
    qp_input.vd_d.segment(6, nj) = 150 * (joint_d - rs.position().segment(6, nj)) - 10 * rs.velocity().segment(6, nj);

    // send messages
    QPInput2VectorXd(qp_input, &QP_INPUT);
    HumanoidStatus2VectorXd(rs, &STATE);
    state_pub.SetValue(STATE);
    qp_input_pub.SetValue(QP_INPUT);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // call controller
    mydiagram.EvalOutput(*context, output.get());

    // parse output / fwd sim, qp_output will be sent through LCM as well.
    VectorXd2QPOutput(output->get_mutable_port(0)->GetMutableVectorData()->get_mutable_value(), &qp_output);

    STATE[time_idx] += dt;
    integrate_state_rpy(dt, q, qd, qp_output.vd);
    trq = qp_output.joint_torque;
    l_ft = qp_output.foot_wrench_in_sensor_frame[0];
    r_ft = qp_output.foot_wrench_in_sensor_frame[1];
    VectorXd2HumanoidStatus(STATE, &rs);

    std::cout << "time: " << rs.time() << " com: " << rs.com().transpose() << std::endl;
  }

  return 0;
}




void test_lcm_sub() {
  lcm::LCM lcm;
  // lcm receiver
  drake::systems::lcm::LcmReceiveThread lcm_receive_thread(&lcm);

  // lcm pub
  MessagePublisher pub("test", &lcm, 3);
  pub.Start();

  std::string channel = "test";

  const drake::systems::lcm::TranslatorBetweenLcmtDrakeSignal translator(3);

  drake::systems::lcm::LcmSubscriberSystem dut(channel, translator, &lcm);
  std::unique_ptr<drake::systems::ContextBase<double>> context = dut.CreateDefaultContext();
  std::unique_ptr<drake::systems::SystemOutput<double>> output = dut.AllocateOutput(*context);

  Eigen::VectorXd mlgb;

  while (true) {
    dut.EvalOutput(*context.get(), output.get());
    // Gets the output of the LcmSubscriberSystem.
    // Downcasts the output vector to be a pointer to a BasicVector.
    const drake::systems::BasicVector<double>& basic_vector =
        dynamic_cast<const drake::systems::BasicVector<double>&>(*output->get_port(0).get_vector_data());

    mlgb = basic_vector.get_value();
    std::cout << mlgb.transpose() << std::endl;
  }
}

