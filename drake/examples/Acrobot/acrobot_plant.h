#pragma once

#include <memory>

#include "drake/examples/Acrobot/gen/acrobot_state_vector.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/affine_system.h"

namespace drake {
namespace examples {
namespace acrobot {

/// The Acrobot - a canonical underactuated system as described in <a
/// href="http://underactuated.mit.edu/underactuated.html?chapter=3">Chapter 3
/// of Underactuated Robotics</a>.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
template <typename T>
class AcrobotPlant : public systems::LeafSystem<T> {
 public:
  AcrobotPlant();

  // Non-copyable.
  AcrobotPlant(const AcrobotPlant<T>&) = delete;
  AcrobotPlant& operator=(const AcrobotPlant<T>&) = delete;

  /// The input force to this system is not direct feedthrough.
  bool has_any_direct_feedthrough() const override { return false; }

  ///@{
  /// Manipulator equation of Acrobot: H * qdotdot + C = B*u.
  /// H[2x2] is the mass matrix.
  /// C[2x1] includes the Coriolis term, gravity term and the damping term, i.e.
  /// C[2x1] = Coriolis(q,v)*v + g(q) + [b1*theta1;b2*theta2]
  Vector2<T> VectorC(const AcrobotStateVector<T>& x) const;
  Matrix2<T> MatrixH(const AcrobotStateVector<T>& x) const;
  ///@}

  // getters for robot parameters
  T m1() const { return m1_; }
  T m2() const { return m2_; }
  T l1() const { return l1_; }
  T l2() const { return l2_; }
  T lc1() const { return lc1_; }
  T lc2() const { return lc2_; }
  T Ic1() const { return Ic1_; }
  T Ic2() const { return Ic2_; }
  T b1() const { return b1_; }
  T b2() const { return b2_; }
  T g() const { return g_; }

 protected:
  T DoCalcKineticEnergy(const systems::Context<T>& context) const override;
  T DoCalcPotentialEnergy(const systems::Context<T>& context) const override;

 private:
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  // LeafSystem<T> override.
  std::unique_ptr<systems::ContinuousState<T>> AllocateContinuousState()
      const override;

  // LeafSystem<T> override.
  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::OutputPortDescriptor<T>& descriptor) const override;

  // System<T> override.
  AcrobotPlant<AutoDiffXd>* DoToAutoDiffXd() const override;

  // TODO(russt): Declare these as parameters in the context.

  const double m1_{1.0},  // Mass of link 1 (kg).
      m2_{1.0},           // Mass of link 2 (kg).
      l1_{1.0},           // Length of link 1 (m).
      l2_{2.0},           // Length of link 2 (m).
      lc1_{0.5},   // Vertical distance from shoulder joint to center of mass of
                   // link 1 (m).
      lc2_{1.0},   // Vertical distance from elbox joint to center of mass of
                   // link 2 (m).
      Ic1_{.083},  // Inertia of link 1 about the center of mass of link 1
                   // (kg*m^2).
      Ic2_{.33},   // Inertia of link 2 about the center of mass of link 2
                   // (kg*m^2).
      b1_{0.1},    // Damping coefficient of the shoulder joint (kg*m^2/s).
      b2_{0.1},    // Damping coefficient of the elbow joint (kg*m^2/s).
      g_{9.81};    // Gravitational constant (m/s^2).

  /*
  // parameters for the acrobot in the MIT lab
  const double m1_{2.4367},  // Mass of link 1 (kg).
    m2_{0.6178},           // Mass of link 2 (kg).
    l1_{0.5263},           // Length of link 1 (m).
    l2_{0},                // Length of link 2 (m).
    lc1_{1.6738},   // Vertical distance from shoulder joint to center of
                   // mass of link 1 (m).
    lc2_{1.5651},   // Vertical distance from elbox joint to center of mass of
                   // link 2 (m).
    Ic1_{-4.7443},  // Inertia of link 1 about the center of mass of link 1
                   // (kg*m^2).
    Ic2_{-1.0068},  // Inertia of link 2 about the center of mass of link 2
                   // (kg*m^2).
    b1_{0.0320},    // Damping coefficient of the shoulder joint (kg*m^2/s).
    b2_{0.0413},    // Damping coefficient of the elbow joint (kg*m^2/s).
    g_{9.81};       // Gravitational constant (m/s^2).
  */

  // Note that the Spong controller behaves differently on these two sets of
  // parameters. The controller works well on the first set of parameters,
  // which Spong used in his paper. In contrast, it is difficult to find a
  // functional set of gains to stabilize the robot about its upright fixed
  // point using the second set of parameters, which represent a real robot.
  // This difference illustrates the limitations of the Spong controller.

  const double I1_ = Ic1_ + m1_ * lc1_ * lc1_;
  const double I2_ = Ic2_ + m2_ * lc2_ * lc2_;
  const double m2l1lc2_ = m2_ * l1_ * lc2_;  // Quantities that occur often.
};

/// Constructs the Acrobot with (only) encoder outputs.
template <typename T>
class AcrobotWEncoder : public systems::Diagram<T> {
 public:
  explicit AcrobotWEncoder(bool acrobot_state_as_second_output = false);

  const AcrobotPlant<T>* acrobot_plant() const { return acrobot_plant_; }

  AcrobotStateVector<T>* get_mutable_acrobot_state(
      systems::Context<T>* context) const;

 private:
  AcrobotPlant<T>* acrobot_plant_{nullptr};
};

/// Constructs the LQR controller for stabilizing the upright fixed point using
/// default LQR cost matrices which have been tested for this system.
std::unique_ptr<systems::AffineSystem<double>> BalancingLQRController(
    const AcrobotPlant<double>& acrobot);

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
