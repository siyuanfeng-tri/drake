#include <memory>

#include "drake/examples/manipulation_station/manipulation_station.h"
#include "drake/manipulation/robot_bridge/robot_bridge.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace examples {
namespace manipulation_station {

struct DataRecordParams {
  std::string record_root_dir;
  double record_period{0.1};
  double record_start_time{3};
};

struct PeelCommandParams {
  std::string channel_name;
};

struct MegaDiagramStuff {
  std::unique_ptr<systems::Diagram<double>> diagram;
  std::unique_ptr<multibody::MultibodyPlant<double>> controller_plant;
  ManipulationStation<double>* station{};
  manipulation::robot_bridge::RobotBridge* robot_comm{};
  systems::lcm::LcmSubscriberSystem* peel_command_sub{};
  systems::ConstantVectorSource<double>* tool_vel_cmd{};
};

MegaDiagramStuff build_pdc_mega_diagram(
    const std::string& obj_name, const std::string& model_dir,
    bool visualize_point_cloud,
    const std::optional<DataRecordParams>& record_params,
    const std::optional<PeelCommandParams>& peel_cmd_params);

math::RigidTransform<double> CalcRelativeTransform(
    const multibody::MultibodyPlant<double>& plant, const Eigen::VectorXd& x,
    const multibody::Frame<double>& frame_A,
    const multibody::Frame<double>& frame_B);

}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake
