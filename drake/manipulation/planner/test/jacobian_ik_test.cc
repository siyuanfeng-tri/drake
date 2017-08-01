#include "drake/manipulation/planner/constraint_relaxing_ik.h"
#include "drake/manipulation/planner/jacobian_ik.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsers/urdf_parser.h"

#include "drake/manipulation/util/simple_tree_visualizer.h"
#include "drake/lcm/drake_lcm.h"

namespace drake {
namespace manipulation {
namespace planner {

GTEST_TEST(ConstraintRelaxingIkTest, SolveIkFromFk) {
  const std::string kModelPath = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_polytope_collision.urdf");
  std::unique_ptr<RigidBodyTree<double>> iiwa =
      std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      kModelPath, multibody::joints::kFixed, nullptr, iiwa.get());

  const std::string kEndEffectorLinkName = "iiwa_link_ee";
  const RigidBody<double>* end_effector = iiwa->FindBody(kEndEffectorLinkName);
  KinematicsCache<double> cache = iiwa->CreateKinematicsCache();

  //std::default_random_engine rand(30);
  //VectorX<double> q0 = iiwa->getRandomConfiguration(rand);
  VectorX<double> q0 = iiwa->getZeroConfiguration();
  q0[1] += 45. * M_PI / 180.;
  q0[3] -= M_PI / 2.;
  q0[5] += 45. * M_PI / 180.;

  cache.initialize(q0);
  iiwa->doKinematics(cache);

  Isometry3<double> pose0 =
      iiwa->CalcBodyPoseInWorldFrame(cache, *end_effector);

  const int N = 1000;
  const double dt = 1e-3;
  std::vector<double> T(N);
  std::vector<Isometry3<double>> pose_d(N);
  for (int i = 0; i < N; ++i) {
    T[i] = (i + 1) * dt;
    pose_d[i] = pose0;
    pose_d[i].translation()[0] -= 0.1 * (std::cos(T[i] * 2 * M_PI) - 1);
    pose_d[i].translation()[1] += 0.1 * std::sin(T[i] * 2 * M_PI);
  }

  JacobianIk ik(kModelPath, Isometry3<double>::Identity());
  std::vector<VectorX<double>> q_sol;
  VectorX<double> q_nominal = iiwa->getZeroConfiguration();
  RigidBodyFrame<double> EE(
      "EE", iiwa->FindBody(kEndEffectorLinkName),
      Isometry3<double>::Identity());
  ik.Plan(q0, T, pose_d, EE, q_nominal, &q_sol);

  /*
  // viz
  drake::lcm::DrakeLcm lcm;
  manipulation::SimpleTreeVisualizer viz(*iiwa, &lcm);

  for (int i = 0; i < N; ++i) {
    cache.initialize(q_sol[i]);
    iiwa->doKinematics(cache);
    pose0 = iiwa->CalcBodyPoseInWorldFrame(cache, *end_effector);
    viz.visualize(q_sol[i]);
    usleep(dt * 1e6);
  }
  */
}

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
