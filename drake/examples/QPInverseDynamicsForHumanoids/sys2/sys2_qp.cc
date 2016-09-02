#include "sys2_qp.h"
#include "drake/common/drake_path.h"

namespace drake {
namespace systems {

void System2QP::EvalOutput(const ContextBase<double> &context, SystemOutput<double>* output) const {
  DRAKE_ASSERT(context.get_num_input_ports() == 2);
  DRAKE_ASSERT(output->get_num_ports() == 1);

  // get robot status
  const HumanoidStatus &rs = context.get_abstract_input(0)->GetValue<HumanoidStatus>();

  // get qp input
  const AbstractValue* abs_in = context.get_abstract_input(1);
  const QPInput &qp_input = abs_in->GetValue<QPInput>();

  // qp output
  QPOutput qp_output(rs.robot());

  std::cout << qp_input;
  std::cout << context.get_vector_input(0)->get_value() << std::endl;

  if (context.get_vector_input(0)->get_value().hasNaN() ||
      context.get_vector_input(1)->get_value().hasNaN()) {
    std::cout << "System2QP: not valid input\n";
    qp_output.vd().setZero();
    output->GetMutableData(0)->SetValue(qp_output);
    return;
  }

  /*
  qp_input.supports().push_back(SupportElement(*robot_->FindBody("leftFoot")));
  qp_input.support(0).get_mutable_contact_points().push_back(Vector3d(0.2, 0.05, -0.09));
  qp_input.support(0).get_mutable_contact_points().push_back(Vector3d(0.2, -0.05, -0.09));
  qp_input.support(0).get_mutable_contact_points().push_back(Vector3d(-0.05, -0.05, -0.09));
  qp_input.support(0).get_mutable_contact_points().push_back(Vector3d(-0.05, 0.05, -0.09));

  qp_input.supports().push_back(SupportElement(*robot_->FindBody("rightFoot")));
  qp_input.support(1).get_mutable_contact_points().push_back(Vector3d(0.2, 0.05, -0.09));
  qp_input.support(1).get_mutable_contact_points().push_back(Vector3d(0.2, -0.05, -0.09));
  qp_input.support(1).get_mutable_contact_points().push_back(Vector3d(-0.05, -0.05, -0.09));
  qp_input.support(1).get_mutable_contact_points().push_back(Vector3d(-0.05, 0.05, -0.09));
  */

  QPController qp_controller(rs, 4);

  if (qp_controller.Control(rs, qp_input, &qp_output) < 0) {
    throw std::runtime_error("System2QP: QP canot solve\n");
  }

  /*
  std::cout << "===================================\n";
  for (int i = 0; i < rs.position().size(); i++) {
    std::cout << rs.robot().getPositionName(i) << ": " << rs.position()[i] << ", " << rs.velocity()[i] << std::endl;
  }
  std::cout << "===================================\n";
  */

  //QPOutput2VectorXd(qp_output, &tmp);
  //output->GetMutableVectorData(0)->get_mutable_value() = tmp;
  output->GetMutableData(0)->SetValue(qp_output);
}


}
}
