cmake_minimum_required(VERSION 3.0.2)
project(hover_thrust_estimator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  mavros_msgs
  message_generation
)
find_package(xgc2_observer REQUIRED CONFIG)
find_package(xgc2_state_machine REQUIRED CONFIG)

add_message_files(
  FILES
  HoverThrustEstimate.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hover_thrust_estimator_math hover_thrust_estimator_core
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs mavros_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(hover_thrust_estimator_math
  src/hover_thrust_estimator_runtime.cpp
  src/state_machine/airborne_state.cpp
  src/state_machine/fault_state.cpp
  src/state_machine/ground_state.cpp
  src/state_machine/health_monitor_state.cpp
  src/state_machine/self_check_state.cpp
)
target_compile_features(hover_thrust_estimator_math PUBLIC cxx_std_17)
target_link_libraries(hover_thrust_estimator_math
  xgc2_observer::observer
  xgc2_state_machine::state_machine
)

add_library(hover_thrust_estimator_core
  src/input/hover_thrust_input_producer.cpp
  src/output/hover_thrust_output_consumer.cpp
  src/output/output_event_consumer.cpp
  src/hover_thrust_estimator_node.cpp
)
add_dependencies(hover_thrust_estimator_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
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
  catkin_add_gtest(${PROJECT_NAME}_core_test
    test/hover_thrust_estimator_test.cpp
  )
  if(TARGET ${PROJECT_NAME}_core_test)
    target_compile_features(${PROJECT_NAME}_core_test PUBLIC cxx_std_17)
    target_link_libraries(${PROJECT_NAME}_core_test
      hover_thrust_estimator_math
      xgc2_observer::observer
      xgc2_state_machine::state_machine
      ${catkin_LIBRARIES}
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

install(DIRECTORY msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
)
