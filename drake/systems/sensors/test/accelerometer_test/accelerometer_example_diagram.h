#pragma once

#include <memory>

#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant_that_publishes_xdot.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcmt_drake_signal_translator.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/systems/sensors/accelerometer.h"

namespace drake {
namespace systems {
namespace sensors {

/// TODO
class AccelerometerExampleDiagram : public Diagram<double> {
 public:
  explicit AccelerometerExampleDiagram(::drake::lcm::DrakeLcmInterface* lcm);

  /// Initializes this diagram.
  ///
  /// @param[in] visualizer The visualizer to include with the diagram. This can
  /// be nullptr.
  void Initialize(std::unique_ptr<DrakeVisualizer> visualizer = nullptr);

  /// @name Accessors
  //@{
  const RigidBodyTree<double>& get_tree() const { return *tree_; }

  int get_model_instance_id() const { return model_instance_id_; }

  const RigidBodyFrame<double>& get_sensor_frame() const {
    return *sensor_frame_.get();
  }

  const SignalLogger<double>& get_signal_logger() const {
    return *signal_logger_;
  }
  //@}

 private:
  ::drake::lcm::DrakeLcmInterface* lcm_;
  DiagramBuilder<double> builder_;
  RigidBodyTree<double>* tree_{nullptr};
  int model_instance_id_{};
  std::shared_ptr<RigidBodyFrame<double>> sensor_frame_;
  RigidBodyPlantThatPublishesXdot<double>* plant_{nullptr};
  std::unique_ptr<lcm::LcmtDrakeSignalTranslator> translator_;
  lcm::LcmSubscriberSystem* lcm_subscriber_{nullptr};
  Accelerometer* accelerometer_{nullptr};
  SignalLogger<double>* signal_logger_{nullptr};
  DrakeVisualizer* visualizer_{nullptr};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
