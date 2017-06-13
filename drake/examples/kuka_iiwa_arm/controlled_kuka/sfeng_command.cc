#include "drake/common/drake_path.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/iiwa_controller.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"

#include "drake/manipulation/util/motion_plan_translator.h"
#include "drake/lcmt_motion_plan.hpp"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

using namespace qp_inverse_dynamics;

constexpr int kDim = 14;

constexpr double box_x = 0.54;

void Approach(const RigidBodyTree<double>& robot, ::lcm::LCM *lcm) {
  VectorX<double> q1(kDim);
  // left firt.
  q1 << 0, 5, 0, -60, 0, 25, 0,
        0, 5, 0, -60, 0, 25, 0;

  for (int i = 0; i < 14; i++) {
    q1[i] = q1[i] / 180. * M_PI;
  }

  VectorX<double> zero = VectorX<double>::Zero(kDim);
  double duration = 2;
  std::vector<double> times = {0, duration};
  std::vector<VectorX<double>> qs(2, zero);
  std::vector<VectorX<double>> vs(2, zero);
  std::vector<VectorX<double>> vds(2, zero);
  qs.back() = q1;

  lcmt_motion_plan msg;
  manipulation::MotionPlanTranslator::InitializeMessage(&msg);
  msg.timestamp = static_cast<int64_t>(time(NULL));

  manipulation::DofTrajectoryTranslator::EncodeMessage(
      times, qs, vs, vds,
      &msg.dof_motion);

  lcm->publish("IIWA_PLAN", &msg);
  usleep(6 * duration * 1e6);
}

void MoveStraight(
    const RigidBodyTree<double>& robot,
    const param_parsers::ParamSet& params,
    const param_parsers::RigidBodyTreeAliasGroups<double>& alias_groups,
    ::lcm::LCM *lcm) {
  VectorX<double> q1(kDim);
  q1 << 0, 5, 0, -60, 0, 25, 0,
        0, 5, 0, -60, 0, 25, 0;

  for (int i = 0; i < kDim; i++) {
    q1[i] = q1[i] / 180. * M_PI;
  }

  lcmt_motion_plan msg;
  manipulation::MotionPlanTranslator::InitializeMessage(&msg);
  msg.timestamp = static_cast<int64_t>(time(NULL));

  ///////////////////
  VectorX<double> zero = VectorX<double>::Zero(kDim);
  double duration = 1;
  std::vector<double> times = {0, duration};
  std::vector<VectorX<double>> qs(2, q1);
  std::vector<VectorX<double>> vs(2, zero);
  std::vector<VectorX<double>> vds(2, zero);

  manipulation::DofTrajectoryTranslator::EncodeMessage(
      times, qs, vs, vds,
      &msg.dof_motion);

  ///////////////////
  msg.num_body_motions = 2;
  msg.body_motions.resize(msg.num_body_motions);

  int body_idx = 0;
  {
    const RigidBody<double>* body = robot.FindBody("right_iiwa_link_ee");
    KinematicsCache<double> cache = robot.CreateKinematicsCache();
    cache.initialize(q1, zero);
    robot.doKinematics(cache, true);

    std::vector<Isometry3<double>> poses(2);
    std::vector<Vector6<double>> velocities(2, Vector6<double>::Zero());
    std::vector<Vector6<double>> accelerations(2, Vector6<double>::Zero());

    poses[0] = robot.CalcBodyPoseInWorldFrame(cache, *body);
    std::cout << poses[0].translation()[0] << std::endl;
    poses[1] = poses[0];
    // Move in +x.
    poses[1].translation()[0] = box_x + 0.01;

    manipulation::CartesianTrajectoryTranslator::EncodeMessage(
        body->get_name(),
        times, poses, velocities, accelerations, &(msg.body_motions[body_idx++]));
  }

  {
    const RigidBody<double>* body = robot.FindBody("left_iiwa_link_ee");
    KinematicsCache<double> cache = robot.CreateKinematicsCache();
    cache.initialize(q1, zero);
    robot.doKinematics(cache, true);

    std::vector<Isometry3<double>> poses(2);
    std::vector<Vector6<double>> velocities(2, Vector6<double>::Zero());
    std::vector<Vector6<double>> accelerations(2, Vector6<double>::Zero());

    poses[0] = robot.CalcBodyPoseInWorldFrame(cache, *body);
    std::cout << poses[0].translation()[0] << std::endl;
    poses[1] = poses[0];
    // Move in +x.
    poses[1].translation()[0] = box_x + 0.2 - 0.01;

    manipulation::CartesianTrajectoryTranslator::EncodeMessage(
        body->get_name(),
        times, poses, velocities, accelerations, &(msg.body_motions[body_idx++]));
  }

  lcm->publish("IIWA_PLAN", &msg);
  usleep(6 * duration * 1e6);
}

void MakeContact(const RigidBodyTree<double>& robot,
    const param_parsers::ParamSet& params,
    const param_parsers::RigidBodyTreeAliasGroups<double>& alias_groups,
    ::lcm::LCM *lcm) {
  VectorX<double> q1(kDim);
  q1 << 0, 5, 0, -60, 0, 25, 0,
        0, 5, 0, -60, 0, 25, 0;
  for (int i = 0; i < kDim; i++) {
    q1[i] = q1[i] / 180. * M_PI;
  }

  VectorX<double> zero = VectorX<double>::Zero(kDim);
  double duration = 2;
  std::vector<double> times = {0, duration};
  std::vector<VectorX<double>> qs(2, q1);
  std::vector<VectorX<double>> vs(2, zero);
  std::vector<VectorX<double>> vds(2, zero);

  lcmt_motion_plan msg;
  manipulation::MotionPlanTranslator::InitializeMessage(&msg);
  msg.timestamp = static_cast<int64_t>(time(NULL));

  manipulation::DofTrajectoryTranslator::EncodeMessage(
      times, qs, vs, vds,
      &msg.dof_motion);

  int body_idx = 0;
  msg.num_contact_states = 1;
  msg.contact_states.resize(msg.num_contact_states);
  msg.contact_states.front().timestamp = 0;
  msg.contact_states.front().num_bodies_in_contact = 2;
  msg.contact_states.front().bodies_in_contact.resize(msg.contact_states.front().num_bodies_in_contact);

  {
    ContactInformation contact = params.MakeContactInformation(
        *robot.FindBody("left_iiwa_link_ee"));
    contact.mutable_desired_force() = Vector3<double>(50, 0, 0);
    contact.mutable_desired_force_weight() = Vector3<double>(0.001, 0, 0);

    EncodeContactInformation(contact, &(msg.contact_states.front().bodies_in_contact[body_idx++]));
  }

  {
    ContactInformation contact = params.MakeContactInformation(
        *robot.FindBody("right_iiwa_link_ee"));
    contact.mutable_desired_force() = Vector3<double>(-50, 0, 0);
    contact.mutable_desired_force_weight() = Vector3<double>(0.001, 0, 0);

    EncodeContactInformation(contact, &(msg.contact_states.front().bodies_in_contact[body_idx++]));
  }

  lcm->publish("IIWA_PLAN", &msg);
  usleep(6 * duration * 1e6);
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  const std::string kModelPath =
    "/manipulation/models/iiwa_description/urdf/"
    "sfeng_dual_iiwa14_polytope_collision.urdf";

  const std::string kAliasGroupsPath =
    drake::GetDrakePath() +
    "/examples/QPInverseDynamicsForHumanoids/"
    "config/sfeng_iiwa.alias_groups";

  const std::string kControlConfigPath =
    drake::GetDrakePath() +
    "/examples/QPInverseDynamicsForHumanoids/"
    "config/sfeng_iiwa.id_controller_config";

  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::examples::kuka_iiwa_arm::CreateTreedFromFixedModelAtPose(kModelPath, tree.get());

  ::lcm::LCM lcm;

  drake::examples::kuka_iiwa_arm::Approach(*tree, &lcm);

  drake::examples::qp_inverse_dynamics::param_parsers::RigidBodyTreeAliasGroups<double> alias_groups(*tree);
  alias_groups.LoadFromFile(kAliasGroupsPath);

  drake::examples::qp_inverse_dynamics::param_parsers::ParamSet paramset;
  paramset.LoadFromFile(kControlConfigPath, alias_groups);

  drake::examples::kuka_iiwa_arm::MoveStraight(*tree, paramset, alias_groups, &lcm);

  drake::examples::kuka_iiwa_arm::MakeContact(*tree, paramset, alias_groups, &lcm);

  return 0;
}
