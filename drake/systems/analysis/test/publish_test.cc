#include "drake/systems/analysis/simulator.h"

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace systems {

class PubA : public LeafSystem<double> {
 public:
  PubA() {
    this->DeclarePublishPeriodSec(1e-3);
  }

  void DoPublish(const Context<double>& context) const override {
    ctr_++;
    std::cout << "pubA: " << ctr_ << " @ " << context.get_time() << std::endl;
  }

  void DoCalcOutput(const Context<double>& context,
      SystemOutput<double>* output) const override {}

 private:
  mutable int ctr_ = 0;
};


class PubB : public LeafSystem<double> {
 public:
  PubB() {
    this->DeclarePublishPeriodSec(2e-3);
  }

  void DoPublish(const Context<double>& context) const override {
    ctr_++;
    std::cout << "\tpubB: " << ctr_ << " @ " << context.get_time() << std::endl;
  }

  void DoCalcOutput(const Context<double>& context,
      SystemOutput<double>* output) const override {}

 private:
  mutable int ctr_ = 0;
};

int aa() {
  DiagramBuilder<double> builder;
  builder.AddSystem<PubA>();
  builder.AddSystem<PubB>();
  auto sys = builder.Build();

  Simulator<double> sim(*sys);
  sim.Initialize();

  // sim.set_publish_every_time_step(false);

  sim.StepTo(1e-2);



  return 0;
}

}
}

int main() {
  return drake::systems::aa();
}
