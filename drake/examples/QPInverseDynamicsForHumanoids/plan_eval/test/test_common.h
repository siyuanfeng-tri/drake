#pragma once

#include <gtest/gtest.h>

#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/generic_plan.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

class GenericPlanTest : public ::testing::Test {
 protected:
  void Initialize(
      const std::string& alias_groups_path,
      const std::string& control_param_path) {
    alias_groups_ =
        std::make_unique<param_parsers::RigidBodyTreeAliasGroups<double>>(
            *robot_);
    alias_groups_->LoadFromFile(alias_groups_path);

    params_ = std::make_unique<param_parsers::ParamSet>();
    params_->LoadFromFile(control_param_path, *alias_groups_);

    robot_status_ = std::make_unique<HumanoidStatus>(*robot_, *alias_groups_);
    double time_now = 0.2;
    std::default_random_engine generator(123);
    std::normal_distribution<double> normal;
    VectorX<double> q = robot_->getRandomConfiguration(generator);
    VectorX<double> v(robot_->get_num_velocities());
    for (int i = 0; i < v.size(); i++) {
      v[i] = normal(generator);
    }
    robot_status_->UpdateKinematics(time_now, q, v);

    // Initializes the plan.
    dut_->Initialize(*robot_status_, *params_, *alias_groups_);
  }

  std::unique_ptr<RigidBodyTree<double>> robot_;
  std::unique_ptr<param_parsers::RigidBodyTreeAliasGroups<double>>
      alias_groups_;
  std::unique_ptr<param_parsers::ParamSet> params_;
  std::unique_ptr<HumanoidStatus> robot_status_;

  std::unique_ptr<GenericPlan<double>> dut_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
