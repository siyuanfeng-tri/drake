
add_definitions(-g)
set(qp_inverse_dynamics_controller_src
  control_utils.cc
  humanoid_status.cc
  example_qp_input_for_valkyrie.cc
  qp_controller.cc)

add_library(drakeQPController ${qp_inverse_dynamics_controller_src})
target_link_libraries(drakeQPController
  drakeOptimization
  drakeRBM
  drakeSide
  drakeTrajectories)

add_library(drakePlanEval generic_plan.cc manip_plan.cc walking_plan.cc)
target_link_libraries(drakePlanEval
  drakeQPController
  drakeTrajectories
  drakeZMPPlanner
  drakeTrajectories)

if(lcm_FOUND)
  add_library(drakeQPControllerLcmTranslator lcm_utils.cc)
  target_link_libraries(drakeQPControllerLcmTranslator
    drakeRBM
    drakeLCMTypes
    drakeLCMUtil
    lcmtypes_bot2-core-cpp)
endif()

add_subdirectory(system)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
