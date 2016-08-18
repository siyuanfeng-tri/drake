#pragma once

#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/common/drake_assert.h"
#include <Eigen/Geometry>
#include <iostream>


// for the rpy version, this is actually unnecessary.
// q += v*dt;  v += vd*dt
// rpy angles are absolute rotations, v = qd (in world frame)
template<typename DerivedQ, typename DerivedV, typename DerivedVd> void integrate_state_rpy(double dt, Eigen::MatrixBase<DerivedQ> &q, Eigen::MatrixBase<DerivedV> &v, const Eigen::MatrixBase<DerivedVd> &vd) {
  DRAKE_ASSERT(q.rows() == v.rows() && v.rows() == vd.rows());
  DRAKE_ASSERT(q.cols() == v.cols() && v.cols() == vd.cols() && v.cols() == 1);

  Eigen::Vector3d rpy = q.block(3,0,3,1);
  Eigen::Matrix3d rot = drake::math::rpy2rotmat(rpy);
  Eigen::Vector3d wdt = v.block(3,0,3,1) * dt;
  Eigen::AngleAxisd wdt_rot(wdt.norm(), wdt.normalized());

  q += v * dt;
  if (wdt.norm() > 1e-12)
    q.block(3,0,3,1) = drake::math::rotmat2rpy(wdt_rot * rot);

  v += vd * dt;
}

