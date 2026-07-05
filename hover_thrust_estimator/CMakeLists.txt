cmake_minimum_required(VERSION 3.0.2)
project(hover_thrust_estimator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  mavros_msgs
  ros1_utils
  state_machine_msgs
  hover_thrust_estimator_msgs
)
find_package(xgc2_math REQUIRED CONFIG)
find_package(xgc2_state_machine REQUIRED CONFIG)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hover_thrust_estimator_math hover_thrust_estimator_core
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs mavros_msgs ros1_utils state_machine_msgs hover_thrust_estimator_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(hover_thrust_estimator_math
  src/hover_thrust_estimator_runtime.cpp
  src/state_machine/airborne_state.cpp
  src/state_machine/ground_state.cpp
  src/state_machine/health_monitor_state.cpp
  src/state_machine/self_check_state.cpp
)
target_compile_features(hover_thrust_estimator_math PUBLIC cxx_std_17)
target_link_libraries(hover_thrust_estimator_math
  xgc2_math::math
  xgc2_state_machine::state_machine
)

add_library(hover_thrust_estimator_core
  src/input/hover_thrust_input_producer.cpp
  src/output/hover_thrust_debug_trace_consumer.cpp
  src/output/hover_thrust_output_consumer.cpp
  src/hover_thrust_estimator_node.cpp
)
add_dependencies(hover_thrust_estimator_core ${catkin_EXPORTED_TARGETS})
target_compile_features(hover_thrust_estimator_core PUBLIC cxx_std_17)
target_link_libraries(hover_thrust_estimator_core
  hover_thrust_estimator_math
  xgc2_state_machine::state_machine
  ${catkin_LIBRARIES}
)

add_executable(hover_thrust_estimator_node
  src/hover_thrust_estimator_main.cpp
)
add_dependencies(hover_thrust_estimator_node ${catkin_EXPORTED_TARGETS})
target_compile_features(hover_thrust_estimator_node PUBLIC cxx_std_17)
target_link_libraries(hover_thrust_estimator_node
  hover_thrust_estimator_core
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_add_gtest(${PROJECT_NAME}_core_test
    test/hover_thrust_estimator_test.cpp
  )
  if(TARGET ${PROJECT_NAME}_core_test)
    target_compile_features(${PROJECT_NAME}_core_test PUBLIC cxx_std_17)
    target_link_libraries(${PROJECT_NAME}_core_test
      hover_thrust_estimator_math
      xgc2_math::math
      xgc2_state_machine::state_machine
      ${catkin_LIBRARIES}
    )
    set_target_properties(${PROJECT_NAME}_core_test PROPERTIES
      BUILD_RPATH "\$ORIGIN/.."
      LINK_FLAGS "-Wl,--disable-new-dtags"
    )
  endif()

  add_rostest_gtest(${PROJECT_NAME}_node_params_test
    test/hover_thrust_estimator_node_params.test
    test/hover_thrust_estimator_node_params_test.cpp
  )
  if(TARGET ${PROJECT_NAME}_node_params_test)
    add_dependencies(${PROJECT_NAME}_node_params_test ${catkin_EXPORTED_TARGETS})
    target_compile_features(${PROJECT_NAME}_node_params_test PUBLIC cxx_std_17)
    target_link_libraries(${PROJECT_NAME}_node_params_test
      hover_thrust_estimator_core
      ${catkin_LIBRARIES}
    )
    set_target_properties(${PROJECT_NAME}_node_params_test PROPERTIES
      BUILD_RPATH "\$ORIGIN/.."
      LINK_FLAGS "-Wl,--disable-new-dtags"
    )
  endif()
endif()

install(TARGETS hover_thrust_estimator_math hover_thrust_estimator_core hover_thrust_estimator_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
