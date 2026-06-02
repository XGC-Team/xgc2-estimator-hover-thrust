cmake_minimum_required(VERSION 3.0.2)
project(hover_thrust_estimator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  mavros_msgs
)

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
target_link_libraries(hover_thrust_estimator_core ${catkin_LIBRARIES})

add_executable(hover_thrust_estimator_node
  src/hover_thrust_estimator_main.cpp
)
add_dependencies(hover_thrust_estimator_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(hover_thrust_estimator_node
  hover_thrust_estimator_core
  ${catkin_LIBRARIES}
)
