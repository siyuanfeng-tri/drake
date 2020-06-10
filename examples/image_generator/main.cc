#include <ctime>
#include <iomanip>
#include <random>
#include <sstream>
#include <experimental/filesystem>
#include <gflags/gflags.h>

#include "drake/examples/image_generator/image_generator.h"
#include "drake/math/random_rotation.h"
#include "drake/math/roll_pitch_yaw.h"

DEFINE_double(min_xy, -0, "");
DEFINE_double(max_xy, 0, "");
DEFINE_double(min_z, 0.3, "");
DEFINE_double(max_z, 0.6, "");
DEFINE_int32(num_images, 1, "");
DEFINE_int32(mask_value, -1, "");
DEFINE_double(min_r, 0., "");
DEFINE_double(max_r, 0., "");
DEFINE_double(min_p, 0., "");
DEFINE_double(max_p, 0., "");
DEFINE_double(min_y, 0., "");
DEFINE_double(max_y, 0., "");

DEFINE_string(model_path, "", "Path to model");
DEFINE_string(output_dir, "/home/sfeng/tmp/image_generator/",
              "output directory");

namespace drake {
namespace examples {
namespace image_generator {
namespace {

using math::RigidTransform;
std::string GetExperimentRootDir() {
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%d-%H-%M-%S");
  std::string output_dir = FLAGS_output_dir;
  if (output_dir.back() != '/') output_dir += "/";
  return output_dir + oss.str();
}

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  const std::string output_root = GetExperimentRootDir();
  ImageGenerator gen(FLAGS_model_path, output_root, FLAGS_mask_value);
  auto context = gen.CreateDefaultContext();

  auto timed_events = gen.AllocateCompositeEventCollection();
  gen.CalcNextUpdateTime(*context, timed_events.get());
  DRAKE_DEMAND(timed_events->HasPublishEvents());

  const auto& publish_events = timed_events->get_publish_events();
  DRAKE_DEMAND(publish_events.HasEvents());

  math::RigidTransform<double> X_WO(Eigen::Vector3d(0, 0, 0.));
  gen.SetObjectPose(X_WO, context.get());

  std::default_random_engine generator(
      std::chrono::system_clock::now().time_since_epoch().count());
  std::uniform_real_distribution<double> z_dist(FLAGS_min_z, FLAGS_max_z);
  std::uniform_real_distribution<double> xy_dist(FLAGS_min_xy, FLAGS_max_xy);

  std::uniform_real_distribution<double> r_dist(FLAGS_min_r * M_PI / 180.,
                                                FLAGS_max_r * M_PI / 180.);
  std::uniform_real_distribution<double> p_dist(FLAGS_min_p * M_PI / 180.,
                                                FLAGS_max_p * M_PI / 180.);
  std::uniform_real_distribution<double> y_dist(FLAGS_min_y * M_PI / 180.,
                                                FLAGS_max_y * M_PI / 180.);

  for (int i = 0; i < FLAGS_num_images;) {
    const Eigen::Vector3d xyz(xy_dist(generator), xy_dist(generator),
                              -z_dist(generator));
    // drake::log()->info("xyz: {}", xyz.transpose());
    //    const Eigen::Quaterniond rand_q =
    //      math::UniformlyRandomQuaternion(&generator);

    // const math::RollPitchYaw rand_q(r_dist(generator), p_dist(generator),
    //                        y_dist(generator));

    math::RigidTransform<double> X_OC =
        math::RigidTransform<double>(
            math::RollPitchYaw<double>(r_dist(generator), 0, 0),
            Eigen::Vector3d::Zero()) *
        math::RigidTransform<double>(
            math::RollPitchYaw<double>(0, p_dist(generator), 0),
            Eigen::Vector3d::Zero()) *
        math::RigidTransform<double>(
            math::RollPitchYaw<double>(0, 0, y_dist(generator)),
            Eigen::Vector3d::Zero()) *
        math::RigidTransform<double>(xyz);

    gen.SetCameraPose(X_WO * X_OC, context.get()),
        gen.Publish(*context, publish_events);
    i++;
  }

  return 0;
}

}  // namespace
}  // namespace image_generator
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::image_generator::do_main(argc, argv);
}
