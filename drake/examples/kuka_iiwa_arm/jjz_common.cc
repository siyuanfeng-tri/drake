#include "drake/examples/kuka_iiwa_arm/jjz_common.h"

namespace drake {
namespace examples {
namespace jjz {

const Matrix3<double> R_ET(AngleAxis<double>(M_PI, Vector3<double>::UnitY()).toRotationMatrix());
const Isometry3<double> X_ET(Eigen::Translation<double, 3>(Vector3<double>(0, 0, 0.15)) * Isometry3<double>(R_ET));
const Isometry3<double> X_WG(Eigen::Translation<double, 3>(Vector3<double>(0.5, 0.2, 0.)) * AngleAxis<double>(-M_PI / 2., Vector3<double>::UnitZ()));

}  // namespace jjz
}  // namespace examples
}  // namespace drake
