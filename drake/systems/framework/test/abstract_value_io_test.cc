#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/value.h"
#include "drake/systems/framework/diagram_builder.h"
#include "gtest/gtest.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/primitives/constant_value_source.h"

#include <string>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <iomanip>

struct Input {
  double a;
  int b;
};

struct Output {
  std::string str;
  int c;
};

namespace drake {
namespace systems {
namespace {


template <typename T> class MySystem : public LeafSystem<T> {
 public:
  explicit MySystem() {
    this->DeclareAbstractInputPort(kInheritedSampling);
    this->DeclareAbstractOutputPort(kInheritedSampling);
  }

  void EvalOutput(const ContextBase<T>& context, SystemOutput<T>* output) const {
    const AbstractValue* abs_in = context.get_abstract_input(0);
    const Input in = abs_in->GetValue<Input>();
    Output out;
    std::ostringstream os;
    os << std::setprecision(3) << in.a;
    out.str = os.str() + std::to_string(in.b);
    out.c = in.a + in.b;
    AbstractValue* abs_out = output->GetMutableData(0);
    abs_out->SetValue(out);
  }

  std::unique_ptr<SystemOutput<T>> AllocateOutput(const ContextBase<T>& context) const {
    std::unique_ptr<LeafSystemOutput<T>> output(new LeafSystemOutput<T>);
    Output out;
    output->add_port(std::unique_ptr<AbstractValue>(new Value<Output>(out)));
    return std::unique_ptr<SystemOutput<T>>(output.release());
  }
};


GTEST_TEST(AbstractValueIOTest, test) {
  DiagramBuilder<double> builder;
  Input in;
  in.a = 3.14;
  in.b = 233;

  std::unique_ptr<System<double>> source = std::make_unique<ConstantValueSource<double>>(std::unique_ptr<AbstractValue>(new Value<Input>(in)));
  MySystem<double> sys;

  // I think source outputs 1 output which has type Input.

  builder.Connect(source->get_output_port(0), sys.get_input_port(0));
  builder.ExportOutput(sys.get_output_port(0));

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  auto output = diagram->AllocateOutput(*context);
  DRAKE_ASSERT(context->get_num_input_ports() == 0);
  DRAKE_ASSERT(output->get_num_ports() == 1);

  diagram->EvalOutput(*context, output.get());
  const Output &out = output->get_port(0).get_abstract_data()->GetValue<Output>();

  std::cout << out.str << " " << out.c << std::endl;

  EXPECT_EQ("3.14233", output->get_port(0).get_abstract_data()->GetValue<Output>().str);
}

}  // namespace
}  // namespace systems
}  // namespace drake

