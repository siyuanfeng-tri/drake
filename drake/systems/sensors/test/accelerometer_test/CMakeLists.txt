if(Bullet_FOUND)
  add_executable(accelerometer_test
      accelerometer_test.cc
      accelerometer_example_diagram.cc)
  target_link_libraries(accelerometer_test
      drakeLcm
      drakeLCMSystem2
      drakeMultibodyParsers
      drakeSensors
      drakeSystemAnalysis
      drakeSystemPrimitives
      GTest::GTest
      GTest::Main)
  drake_add_test(NAME accelerometer_test COMMAND accelerometer_test)
endif()

if(Bullet_FOUND AND lcm_FOUND)
  add_executable(accelerometer_example
      accelerometer_example.cc
      accelerometer_example_diagram.cc)
  target_link_libraries(accelerometer_example
      drakeLcm
      drakeLCMSystem2
      drakeMultibodyParsers
      drakeSensors
      drakeSystemAnalysis
      drakeSystemPrimitives
      GTest::GTest
      GTest::Main)
endif()
