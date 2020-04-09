#pragma once

#include <fstream>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {

/**
 * This is a Drake System block that takes in a state vector, and outputs a
 * drake::lcmt_viewer_draw message that contains information about the frames
 * for visualization.
 *
 * @ingroup rigid_body_systems
 */
class PoseWriter : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PoseWriter)

  /**
   * Constructor.
   * @param tree A const pointer to the MultibodyPlant used for doing
   * kinematics. Its life span must be longer than this.
   * @param local_transforms A vector of frames to be visualized.
   */
  PoseWriter(const MultibodyPlant<double>* plant,
             const std::vector<const Frame<double>*>& frames,
             const std::string& file_name, double period, double t_offset);

 private:
  void WritePose(const systems::Context<double>& context) const;

  const MultibodyPlant<double>& plant_;
  mutable std::unique_ptr<systems::Context<double>> context_;
  const std::vector<const Frame<double>*> frames_;

  mutable int ctr_{0};
  mutable std::ofstream out_;
};

}  // namespace multibody
}  // namespace drake
