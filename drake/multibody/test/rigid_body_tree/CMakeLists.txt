if(Bullet_FOUND)
  drake_add_cc_test(rbt_collisions_test)
  target_link_libraries(rbt_collisions_test drakeRBM)

  drake_add_cc_test(rigid_body_collision_clique_test)
  target_link_libraries(rigid_body_collision_clique_test drakeRBM)
endif()

drake_add_cc_test(rigid_body_tree_test)
target_link_libraries(rigid_body_tree_test drakeRBM)

drake_add_cc_test(rigid_body_tree_inverse_dynamics_test)
target_link_libraries(rigid_body_tree_inverse_dynamics_test drakeRBM)

if (yaml-cpp_FOUND)
  drake_add_cc_test(parser_yaml_test)
  target_link_libraries(parser_yaml_test drakeRBM)
endif()
