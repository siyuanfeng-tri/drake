#include "drake/common/drake_path.h"
#include "qp_controller.h"
#include <chrono>

QPOutput TestGravityCompensation(HumanoidStatus& robot_status) {
  // Make controller.
  QPController con(robot_status);
  QPInput input;
  QPOutput output;
  InitQPOutput(robot_status.robot(), &output);
  InitQPInput(robot_status.robot(), &input);

  // Setup QP controller's parameter.
  con.param.mu = 1;
  con.param.mu_Mz = 0.1;
  con.param.x_max = 0.2;
  con.param.x_min = -0.05;
  con.param.y_max = 0.05;
  con.param.y_min = -0.05;

  // Make input.
  // These represent the desired motions for the robot, and are typically
  // outputs of motion planner or hand-crafted behavior state machines.
  input.comdd_d.setZero();
  input.pelvdd_d.setZero();
  input.torsodd_d.setZero();
  input.footdd_d[Side::LEFT].setZero();
  input.footdd_d[Side::RIGHT].setZero();
  input.wrench_d[Side::LEFT].setZero();
  input.wrench_d[Side::RIGHT].setZero();
  input.vd_d.setZero();
  // [5] is Fz, 660N * 2 is about robot weight.
  input.wrench_d[Side::LEFT][5] = 660;
  input.wrench_d[Side::RIGHT][5] = 660;

  // Weights are set arbitrarily by the control designer, these typically
  // require tuning.
  input.w_com = 1e2;
  input.w_pelv = 1e1;
  input.w_torso = 1e1;
  input.w_foot = 1e1;
  input.w_vd = 1e3;
  input.w_wrench_reg = 1e-5;

  // for sim
  Eigen::VectorXd tmp_q, tmp_qd;
  double dt = 0.002;
  double T = 0;

  // Call QP controller.
  Vector3d com0 = robot_status.com();
  Vector3d com_d = com0 + Vector3d(0.04, 0, 0);
  
  auto begin = std::chrono::high_resolution_clock::now();

  std::cout << "INITIAL COM " << com0.transpose() << std::endl;
  for (int i = 0; i < 1000; i++) {

    input.comdd_d = 20 * (com_d - com0) + 3 * (-robot_status.comd());

    con.Control(robot_status, input, &output);
    
    // fwd simulate
    tmp_q = robot_status.position();
    tmp_qd = robot_status.velocity();

    tmp_q += tmp_qd * dt;
    tmp_qd += output.vd * dt;
    T += dt;

    robot_status.Update(T, tmp_q, tmp_qd, output.joint_torque, output.foot_wrench_in_sensor_frame[0], output.foot_wrench_in_sensor_frame[1]); 
  }
  
  std::cout << "FINAL COM " << robot_status.com().transpose() << std::endl;

  auto end = std::chrono::high_resolution_clock::now();
  size_t nano_sec = std::chrono::duration_cast<std::chrono::nanoseconds>(end-begin).count();
  std::cout << (double)nano_sec / 1e6 << " sec." << std::endl;


  // Print quadratic costs for all the terms.
  ComputeQPCost(robot_status, input, output);

  return output;
}

int main() {
  // Loads model.
  std::string urdf =
      drake::GetDrakePath() +
      std::string(
          "/examples/QPInverseDynamicsForHumanoids/valkyrie_sim_drake.urdf");
  HumanoidStatus robot_status(
      std::make_unique<RigidBodyTree>(urdf, DrakeJoint::ROLLPITCHYAW));

  // Sets state and does kinematics.
  VectorXd q(robot_status.robot().number_of_positions());
  VectorXd v(robot_status.robot().number_of_velocities());

  q.setZero();
  v.setZero();

  // These corresponds to a nominal pose for the Valkyrie robot: slightly
  // crouched, arm raised a bit.
  q[robot_status.joint_name_to_position_index().at("rightHipRoll")] = 0.01;
  q[robot_status.joint_name_to_position_index().at("rightHipPitch")] = -0.5432;
  q[robot_status.joint_name_to_position_index().at("rightKneePitch")] = 1.2195;
  q[robot_status.joint_name_to_position_index().at("rightAnklePitch")] =
      -0.7070;
  q[robot_status.joint_name_to_position_index().at("rightAnkleRoll")] = -0.0069;

  q[robot_status.joint_name_to_position_index().at("leftHipRoll")] = -0.01;
  q[robot_status.joint_name_to_position_index().at("leftHipPitch")] = -0.5432;
  q[robot_status.joint_name_to_position_index().at("leftKneePitch")] = 1.2195;
  q[robot_status.joint_name_to_position_index().at("leftAnklePitch")] = -0.7070;
  q[robot_status.joint_name_to_position_index().at("leftAnkleRoll")] = 0.0069;

  q[robot_status.joint_name_to_position_index().at("rightShoulderRoll")] = 1;
  q[robot_status.joint_name_to_position_index().at("rightShoulderYaw")] = 0.5;
  q[robot_status.joint_name_to_position_index().at("rightElbowPitch")] =
      M_PI / 2.;

  q[robot_status.joint_name_to_position_index().at("leftShoulderRoll")] = -1;
  q[robot_status.joint_name_to_position_index().at("leftShoulderYaw")] = 0.5;
  q[robot_status.joint_name_to_position_index().at("leftElbowPitch")] =
      -M_PI / 2.;

  robot_status.Update(0, q, v,
                      VectorXd::Zero(robot_status.robot().actuators.size()),
                      Vector6d::Zero(), Vector6d::Zero());

  QPOutput output = TestGravityCompensation(robot_status);

  // Print results.
  PrintQPOutput(output);

  return 0;
}
