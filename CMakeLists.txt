cmake_minimum_required(VERSION 3.0.2)
project(hover_thrust_estimator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  mavros_msgs
)
find_package(xgc2_observer REQUIRED CONFIG)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hover_thrust_estimator_core
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs mavros_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(hover_thrust_estimator_core
  src/hover_thrust_estimator_node.cpp
)
add_dependencies(hover_thrust_estimator_core ${catkin_EXPORTED_TARGETS})
target_compile_features(hover_thrust_estimator_core PUBLIC cxx_std_17)
target_link_libraries(hover_thrust_estimator_core
  ${catkin_LIBRARIES}
  xgc2_observer::observer
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
      xgc2_observer::observer
      ${catkin_LIBRARIES}
    )
  endif()
endif()

install(TARGETS hover_thrust_estimator_core hover_thrust_estimator_node
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
