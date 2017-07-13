#project (PlanarPushing)
#if(BUILD_TESTING)
#  add_subdirectory(test)
#endif()

add_library_with_exports(LIB_NAME dubinsPushing SOURCE_FILES
  dubins_interface.cpp
  dubins.cpp
  )
target_link_libraries(dubinsPushing Eigen3::Eigen)

drake_install_libraries(dubinsPushing)
drake_install_headers(dubins.h dubins_interface.h)
drake_install_pkg_config_file(dubins-pushing
  TARGET dubinsPushing
  LIBS -ldubinsPushing
  REQUIRES
    drake-common
    drake-system-framework
    eigen3
)

add_executable(demo_dubins_pushing demo_dubins_pushing.cpp)
target_link_libraries(demo_dubins_pushing dubinsPushing)
