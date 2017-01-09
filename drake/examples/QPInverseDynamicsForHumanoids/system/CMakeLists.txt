if(gurobi_FOUND AND lcm_FOUND AND yaml-cpp_FOUND)
  add_executable(valkyrie_balancing_controller_system
    valkyrie_balancing_controller_system.cc)
  target_link_libraries(valkyrie_balancing_controller_system
    drakeLCMSystem2
    drakeMultibodyParsers
    drakeQPController
    drakeQPControllerLcmTranslator
    drakeQPControllerParamParsing
    drakeRigidBodyAliasGroups)
endif()
