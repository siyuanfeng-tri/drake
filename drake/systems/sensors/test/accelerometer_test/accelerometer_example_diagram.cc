#include "drake/systems/sensors/test/accelerometer_test/accelerometer_example_diagram.h"

#include <memory>
#include <string>
#include <utility>

#include "drake/common/drake_path.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/model_instance_id_table.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

using std::make_unique;
using std::move;
using std::unique_ptr;

namespace drake {

using parsers::ModelInstanceIdTable;
using parsers::sdf::AddModelInstancesFromSdfFileToWorld;

namespace systems {

using lcm::LcmSubscriberSystem;
using lcm::LcmtDrakeSignalTranslator;

namespace sensors {

AccelerometerExampleDiagram::AccelerometerExampleDiagram(
    ::drake::lcm::DrakeLcmInterface* lcm)
    : lcm_(lcm) {
  const std::string model_file_name =
      GetDrakePath() + "/systems/sensors/test/accelerometer_test/"
      "sideways_pendulum/sideways_pendulum.sdf";
  const std::string model_name = "Sideways_Pendulum";
  const std::string xdot_channel_name = "xdot_channel";

  auto tree = make_unique<RigidBodyTree<double>>();
  tree_ = tree.get();

  // Adds a box to the RigidBodyTree and obtains its model instance ID.
  const parsers::ModelInstanceIdTable table =
  AddModelInstancesFromSdfFileToWorld(
          model_file_name,
          drake::multibody::joints::kFixed, tree_);
  model_instance_id_ = table.at(model_name);

  // Specifies the location of the accelerometer sensor.
  Eigen::Isometry3d sensor_frame_transform = Eigen::Isometry3d::Identity();
  sensor_frame_transform.translation() << 0, -1, 0.05;

  // Adds a frame to the RigidBodyTree called "sensor frame" that is coincident
  // with the "swing_arm" body within the RigidBodyTree.
  sensor_frame_ = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "sensor frame",
      tree->FindBody("swing_arm"), sensor_frame_transform);
  tree->addFrame(sensor_frame_);

  plant_ =
      builder_.template AddSystem<RigidBodyPlantThatPublishesXdot<double>>(
          move(tree), xdot_channel_name, lcm_);

  translator_ = make_unique<LcmtDrakeSignalTranslator>(
      plant_->get_num_states());
  lcm_subscriber_ =
      builder_.template AddSystem<LcmSubscriberSystem>(xdot_channel_name,
                                                      *translator_, lcm_);

  accelerometer_ = Accelerometer::AttachAccelerometer(
      "my accelerometer",
      *sensor_frame_,
      *plant_,
      true /* include_gravity */,
      &builder_);

  signal_logger_ =
      builder_.template AddSystem<SignalLogger<double>>(
          accelerometer_->get_output_port().size());

  auto constant_zero_source =
      builder_.template AddSystem<ConstantVectorSource<double>>(
          VectorX<double>::Zero(plant_->get_input_port(0).size()));

  builder_.Connect(lcm_subscriber_->get_output_port(0),
                  accelerometer_->get_plant_state_derivative_input_port());
  builder_.Connect(constant_zero_source->get_output_port(),
                  plant_->get_input_port(0));
  builder_.Connect(accelerometer_->get_output_port(),
                  signal_logger_->get_input_port(0));
}

void AccelerometerExampleDiagram::Initialize(
    unique_ptr<DrakeVisualizer> visualizer) {

  if (visualizer != nullptr) {
     visualizer_ = builder_.AddSystem(move(visualizer));
     builder_.Connect(plant_->state_output_port(),
                     visualizer_->get_input_port(0));
  }

  builder_.BuildInto(this);
  lcm_->StartReceiveThread();
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
