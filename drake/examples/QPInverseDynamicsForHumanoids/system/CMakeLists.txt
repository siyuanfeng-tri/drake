#if(lcm_FOUND)
#  set(qp_inverse_dynamics_controller_systems_src
#    manip_plan_eval_system.cc)

#  add_library(drakeQPControllerSystems ${qp_inverse_dynamics_controller_systems_src})
#  target_link_libraries(drakeQPControllerSystems
#    drakeLCMSystem2
#    drakeQPController
#    drakeQPControllerLcmTranslator
#    drakeRigidBodyPlant
#    drakeSystemAnalysis
#    robotlocomotion-lcmtypes-cpp)
#endif()
