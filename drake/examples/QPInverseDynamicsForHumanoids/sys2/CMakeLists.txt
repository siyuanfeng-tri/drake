add_executable(sys2_qp_test sys2_qp_diagram_test.cc sys2_qp.cc ../qp_controller.cc ../rigid_body_tree_utils.cc ../humanoid_status.cc qp_io_utils.cc)
target_link_libraries(sys2_qp_test drakeRBSystem drakeOptimization drakeSide drakeLCMSystem2)
