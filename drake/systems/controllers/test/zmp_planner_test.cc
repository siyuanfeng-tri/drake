#include <iostream>
#include <fstream>

#include "drake/systems/controllers/zmp_planner.h"
#include "drake/systems/controllers/generate_spline.h"

int main() {
  drake::systems::ZMPPlanner zmp;

  std::vector<double> Ts;
  std::vector<Eigen::Vector2d> zmp_d;
  std::ofstream out;

  Eigen::Vector4d x0(0, 0, 0, 0);
  double x = 0;
  double y = 0.5;
  double time = 0;
  double ss = 1;
  double ds = 0.5;
  Ts.push_back(time);
  zmp_d.push_back(x0.head<2>());

  for (int i = 0; i < 5; i++) {
    time += ds;
    x += 0.5;
    y *= -1;
    Ts.push_back(time);
    zmp_d.push_back(Eigen::Vector2d(x, y));

    time += ss;
    Ts.push_back(time);
    zmp_d.push_back(Eigen::Vector2d(x, y));
  }

  Eigen::Vector2d zero(Eigen::Vector2d::Zero());
  PiecewisePolynomial<double> zmp_traj_ = GeneratePCHIPSpline(Ts, zmp_d, zero, zero);
  zmp.Plan(zmp_traj_, x0, 1);

  //std::cout << "S: " << zmp.S_ << std::endl;
  out.open("/home/sfeng/zmp_d");
  for (double t = Ts[0]; t <= Ts[Ts.size()-1] + 5; t+= 0.01) {
    out << t << " " << zmp.GetDesiredZMP(t).transpose() << std::endl;
  }
  out.close();

  out.open("/home/sfeng/com");
  for (double t = Ts[0]; t <= Ts[Ts.size()-1] + 5; t+= 0.01) {
    out << t << " " << zmp.GetNominalCOM(t).transpose() << std::endl;
  }
  out.close();

  /*
  out.open("/home/sfeng/s1");
  for (double t = Ts[0]; t <= Ts[Ts.size()-1] + 5; t+= 0.01) {
    out << t << " " << zmp.s1_traj_.value(t).transpose() << std::endl;
  }
  out.close();
  */

  return 0;
}
