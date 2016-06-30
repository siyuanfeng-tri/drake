#include "QPController.h"

QPOutput testGravityCompensation(const HumanoidState &rs) {
  // make controller
  QPController con;
  QPInput input(*rs.robot);
  QPOutput output(*rs.robot);

  input.comdd_d.setZero();
  input.pelvdd_d.setZero();
  input.torsodd_d.setZero();
  input.footdd_d[Side::LEFT].setZero();
  input.footdd_d[Side::RIGHT].setZero();
  input.qdd_d.setZero();
  input.w_com = 1e2;
  input.w_pelv = 1e1;
  input.w_torso = 1e1;
  input.w_foot = 1e1;
  input.w_qdd = 1e4;
  input.w_wrench_reg = 1e-5;

  ////////////////////////////////////////////////////////////////////
  // call QP
  assert(con.control(rs, input, output) == 0);

  // print result
  output.print();

  // call QP2
  QPController2 con2;
  QPOutput output2(*rs.robot);
  assert(con2.control(rs, input, output2) == 0);
  output2.print();

  std::cout << "output1 == output2 " << (output == output2) << std::endl;

  return output;
}

int main() {
  ////////////////////////////////////////////////////////////////////
  // load model
  std::string urdf = std::string(VALKYRIE_URDF_PATH)
                   + std::string("/valkyrie_sim_drake.urdf");
  HumanoidState rs(
      std::unique_ptr<RigidBodyTree>(
        new RigidBodyTree(urdf, DrakeJoint::ROLLPITCHYAW)));

  ////////////////////////////////////////////////////////////////////
  // set state and do kinematics
  VectorXd q(rs.robot->number_of_positions());
  VectorXd qd(rs.robot->number_of_velocities());

  q.setZero();
  qd.setZero();

  q[rs.jointName2ID.at("rightHipRoll")] = 0.01;
  q[rs.jointName2ID.at("rightHipPitch")] = -0.5432;
  q[rs.jointName2ID.at("rightKneePitch")] = 1.2195;
  q[rs.jointName2ID.at("rightAnklePitch")] = -0.7070;
  q[rs.jointName2ID.at("rightAnkleRoll")] = -0.0069;

  q[rs.jointName2ID.at("leftHipRoll")] = -0.01;
  q[rs.jointName2ID.at("leftHipPitch")] = -0.5432;
  q[rs.jointName2ID.at("leftKneePitch")] = 1.2195;
  q[rs.jointName2ID.at("leftAnklePitch")] = -0.7070;
  q[rs.jointName2ID.at("leftAnkleRoll")] = 0.0069;

  q[rs.jointName2ID.at("rightShoulderRoll")] = 1;
  q[rs.jointName2ID.at("rightShoulderYaw")] = 0.5;
  q[rs.jointName2ID.at("rightElbowPitch")] = M_PI/2.;

  q[rs.jointName2ID.at("leftShoulderRoll")] = -1;
  q[rs.jointName2ID.at("leftShoulderYaw")] = 0.5;
  q[rs.jointName2ID.at("leftElbowPitch")] = -M_PI/2.;

  rs.update(0, q, qd, VectorXd::Zero(rs.robot->actuators.size()),
      Vector6d::Zero(), Vector6d::Zero());

  // test QP controller
  QPOutput output = testGravityCompensation(rs);

  return 0;
}
