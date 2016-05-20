#pragma once

#include "drake/core/Vector.h"
#include "drake/systems/LinearSystem.h"
#include "drake/util/drakeUtil.h"

namespace Drake {

template <typename System>
std::shared_ptr<AffineSystem<NullVector, System::template StateVector,
                             System::template InputVector>>
sfengPD(const System& sys, 
        const Eigen::VectorXd& x0,
        const Eigen::VectorXd& pdGains) {
  const int num_states =
      System::template StateVector<double>::RowsAtCompileTime;
  const int num_inputs =
      System::template InputVector<double>::RowsAtCompileTime;
  
  using namespace std;
  using namespace Eigen;

  cout << "Kp = " << pdGains.segment(0,num_states/2) << endl;
  cout << "Kd = " << pdGains.segment(num_states/2,num_states/2) << endl;

  // todo: return the linear system with the affine transform.  But for now,
  // just give the affine controller:

  // u = Kp(x-x0) + Kd(xd-xd0)
  Matrix<double, 0, 0> nullmat;
  Matrix<double, 0, 1> nullvec;

  Matrix<double, num_inputs, num_states> PD;
  PD.setZero();
  for (int i = 0; i < num_states/2; i++) {
    PD(i,i) = pdGains(i);
    PD(i,i+num_states/2) = pdGains(i+num_states/2);
  }

  return std::make_shared<AffineSystem<NullVector, System::template StateVector,
                                       System::template InputVector>>(
      nullmat, Matrix<double, 0, num_states>::Zero(), nullvec,
      Matrix<double, num_inputs, 0>::Zero(), PD, -PD*toEigen(x0));
}
}
