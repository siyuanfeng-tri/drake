#include "sys2_qp.h"

namespace drake {
namespace systems {

void System2QP::EvalOutput(const ContextBase<double> &context, SystemOutput<double>* output) const {
  DRAKE_ASSERT(context.get_num_input_ports() == 2);
  Eigen::Matrix<double, num_states, 1> X = context.get_vector_input(0)->get_value();
  Eigen::Matrix<double, num_qp_input, 1> qp_input = context.get_vector_input(1)->get_value();

  std::cout << "state: " << X.head(3).transpose() << std::endl;
  std::cout << "qp_in: " << qp_input.head(3).transpose() << std::endl;

  // my fancy math
  Eigen::Matrix<double, num_qdd, 1> qdd;
  qdd.head(3) = X.head(3);
  Eigen::Matrix<double, num_trq, 1> trq;
  trq.head(3) = qp_input.head(3);

  // qdd
  output->get_mutable_port(0)->GetMutableVectorData()->get_mutable_value() = qdd;
  std::cout << "qdd: " << qdd.head(3).transpose() << std::endl;

  // trq
  output->get_mutable_port(1)->GetMutableVectorData()->get_mutable_value() = trq;
  std::cout << "trq: " << trq.head(3).transpose() << std::endl;
  std::cout << "==================\n";
}



void testQP(::lcm::LCM &lcm) {
  // components of this diagram
  // 2 input
  lcm::TranslatorBetweenLcmtDrakeSignal state_trans(System2QP::num_states);
  lcm::TranslatorBetweenLcmtDrakeSignal qp_input_trans(System2QP::num_qp_input);
  lcm::LcmSubscriberSystem state_sub("state", state_trans, &lcm);
  lcm::LcmSubscriberSystem qp_input_sub("qp_input", qp_input_trans, &lcm);
  state_sub.set_name("lcm_to_state");
  qp_input_sub.set_name("lcm_to_qp_input");
  // 2 output
  lcm::TranslatorBetweenLcmtDrakeSignal qdd_trans(System2QP::num_qdd);
  lcm::TranslatorBetweenLcmtDrakeSignal trq_trans(System2QP::num_trq);
  lcm::LcmPublisherSystem qdd_pub("qdd", qdd_trans, &lcm);
  lcm::LcmPublisherSystem trq_pub("trq", trq_trans, &lcm);
  qdd_pub.set_name("qdd_to_lcm");
  trq_pub.set_name("trq_to_lcm");
  // qp controller
  System2QP qp_con;
  qp_con.set_name("qp_controller");

  // connect the diagram
  Diagram<double> mysys;
  mysys.set_name("Sys2 QP diagram");

  mysys.Connect(&state_sub, 0, &qp_con, 0);
  mysys.Connect(&qp_input_sub, 0, &qp_con, 1);
  mysys.Connect(&qp_con, 0, &qdd_pub, 0);
  mysys.Connect(&qp_con, 1, &trq_pub, 0);

  mysys.Finalize();

  std::unique_ptr<ContextBase<double>> context = mysys.CreateDefaultContext();
  std::unique_ptr<SystemOutput<double>> output = mysys.AllocateOutput(*context);

  std::cout << context->get_num_input_ports() << std::endl;
  std::cout << output->get_num_ports() << std::endl;

  while(true) {
    mysys.EvalOutput(*context, output.get());
  }
}



}
}
