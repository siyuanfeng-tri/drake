
#include "drake/common/drake_path.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {

Matrix3<double> vectorToSkewSymmetricSquared(const Vector3<double>& v) {
  Matrix3<double> ret;
  auto a0_2 = v(0) * v(0);
  auto a1_2 = v(1) * v(1);
  auto a2_2 = v(2) * v(2);

  ret(0, 0) = -a1_2 - a2_2;
  ret(0, 1) = v(0) * v(1);
  ret(0, 2) = v(0) * v(2);

  ret(1, 0) = ret(0, 1);
  ret(1, 1) = -a0_2 - a2_2;
  ret(1, 2) = v(1) * v(2);

  ret(2, 0) = ret(0, 2);
  ret(2, 1) = ret(1, 2);
  ret(2, 2) = -a0_2 - a1_2;
  return ret;
};

Matrix3<double> translate_inertia(const Matrix3<double>& I_com, double mass, const Vector3<double>& p, const Vector3<double>& com) {
  Vector3<double> d = com - p;
  return I_com - mass * vectorToSkewSymmetricSquared(d);
}

void combine_inertia(double m0, double m1, const Vector3<double>& com0, const Vector3<double>& com1_b,
    const Matrix3<double>& I0, const Matrix3<double>& I1_b, const Isometry3<double>& X_B0B1,
    Vector3<double>* new_com, Matrix3<double>* new_I) {
  Vector3<double> com1 = X_B0B1 * com1_b;
  Matrix3<double> I1 = X_B0B1.linear() * I1_b * X_B0B1.linear().transpose();

  *new_com = (m0 * com0 + m1 * com1) / (m0 + m1);
  Matrix3<double> new_I0 = translate_inertia(I0, m0, *new_com, com0);
  Matrix3<double> new_I1 = translate_inertia(I1, m1, *new_com, com1);

  *new_I = new_I0 + new_I1;
}

int main() {
  std::unique_ptr<RigidBodyTree<double>> robot = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFile(GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14_simplified_collision.urdf",
  //parsers::sdf::AddModelInstancesFromSdfFile(GetDrakePath() + "/examples/schunk_wsg/models/schunk_wsg_50.sdf",
      multibody::joints::kRollPitchYaw, nullptr, robot.get());

  KinematicsCache<double> cache = robot->CreateKinematicsCache();

  VectorX<double> q = robot->getZeroConfiguration();
  q[0] = 1;
  cache.initialize(q, VectorX<double>::Zero(robot->get_num_velocities()));
  robot->doKinematics(cache);

  MatrixX<double> cen_mom = robot->centroidalMomentumMatrix(cache);

  std::cout << cen_mom.block<6, 6>(0, 0) << std::endl;

  const RigidBody<double>& body = *robot->FindBody("iiwa_link_0");

  Isometry3<double> body_pose = robot->CalcBodyPoseInWorldFrame(cache, body);
  body_pose.translation().setZero();

  std::cout << body_pose.linear() << std::endl;

  Vector3<double> new_com;
  Matrix3<double> new_I;

  combine_inertia(1., 2.,
      Vector3<double>(0, 0, 0), Vector3<double>(0, 0, 0),
      Matrix3<double>::Zero(), Matrix3<double>::Zero(),
      Isometry3<double>::Identity(), &new_com, &new_I);

  std::cout << body.get_spatial_inertia() << std::endl;
  std::cout << body.get_center_of_mass() << std::endl;

  return 0;
}

}


int main(int argc, char* argv[]) {
  return drake::main();
}
