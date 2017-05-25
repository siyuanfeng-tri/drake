#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/manipulation/util/cartesian_trajectory_translator.h"
#include "drake/manipulation/util/dof_trajectory_translator.h"

#include "drake/lcmt_motion_plan.hpp"

namespace drake {
namespace manipulation {

class MotionPlanTranslator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MotionPlanTranslator)

  static void InitializeMessage(lcmt_motion_plan* msg);
};

}  // namespace manipulation
}  // namespace drake
