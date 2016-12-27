if(lcm_FOUND)
  set(system_wrapper_src
    joint_level_controller_system.cc
    plan_eval_system.cc
    qp_controller_system.cc
    robot_state_decoder_system.cc)

  add_library(drakeHumanoidControllerSystemWrappers ${system_wrapper_src})
  target_link_libraries(drakeHumanoidControllerSystemWrappers
    drakeQPController
    drakeQPControllerLcmTranslator
    drakePlanEval
    drakeLCMSystem2)
endif()
