cmake_minimum_required(VERSION 3.16)
project(hover_thrust_estimator VERSION 1.1.24 LANGUAGES CXX)

option(BUILD_ROS "Build the ROS1 transport facade" ON)
option(BUILD_NATIVE "Build the xgc-runtime transport facade" ON)
set(XGC_RUNTIME_SDK_SOURCE_ROOT "" CACHE PATH "Explicit runtime source root for a source SDK build")
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
find_package(xgc2_math REQUIRED CONFIG)
find_package(xgc2_state_machine REQUIRED CONFIG)

add_library(hover_thrust_estimator_math SHARED
  src/hover_thrust_estimator_runtime.cpp
  src/state_machine/airborne_state.cpp src/state_machine/ground_state.cpp
  src/state_machine/health_monitor_state.cpp src/state_machine/self_check_state.cpp)
add_library(HoverThrustEstimator::Runtime ALIAS hover_thrust_estimator_math)
set_target_properties(hover_thrust_estimator_math PROPERTIES EXPORT_NAME Runtime
  VERSION ${PROJECT_VERSION} SOVERSION 0 INSTALL_RPATH "$ORIGIN")
target_compile_features(hover_thrust_estimator_math PUBLIC cxx_std_17)
target_include_directories(hover_thrust_estimator_math PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(hover_thrust_estimator_math PUBLIC
  xgc2_math::math xgc2_state_machine::state_machine)

if(BUILD_NATIVE)
  if(NOT TARGET XgcRuntime::SDK)
    if(XGC_RUNTIME_SDK_SOURCE_ROOT)
      add_subdirectory("${XGC_RUNTIME_SDK_SOURCE_ROOT}/abi" "${CMAKE_CURRENT_BINARY_DIR}/runtime-sdk")
    else()
      find_package(XgcRuntimeSDK CONFIG REQUIRED)
    endif()
  endif()
  add_library(hover_thrust_estimator_native MODULE native/est_hover_thrust.cpp)
  target_link_libraries(hover_thrust_estimator_native PRIVATE
    HoverThrustEstimator::Runtime XgcRuntime::SDK)
  set_target_properties(hover_thrust_estimator_native PROPERTIES
    OUTPUT_NAME est_hover_thrust CXX_VISIBILITY_PRESET hidden
    INSTALL_RPATH "$ORIGIN/../..")
  install(TARGETS hover_thrust_estimator_native
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/xgc-runtime/plugins)
endif()

add_executable(hover_thrust_reference_replay native/reference_replay.cpp)
target_link_libraries(hover_thrust_reference_replay PRIVATE HoverThrustEstimator::Runtime)

if(BUILD_ROS)
  find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs
    mavros_msgs ros1_utils state_machine_msgs hover_thrust_estimator_msgs)
  catkin_package(INCLUDE_DIRS include
    LIBRARIES hover_thrust_estimator_math hover_thrust_estimator_core
    CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs mavros_msgs ros1_utils
      state_machine_msgs hover_thrust_estimator_msgs)
  add_library(hover_thrust_estimator_core SHARED
    src/input/hover_thrust_input_producer.cpp
    src/output/hover_thrust_debug_trace_consumer.cpp
    src/output/hover_thrust_output_consumer.cpp src/hover_thrust_estimator_node.cpp)
  add_dependencies(hover_thrust_estimator_core ${catkin_EXPORTED_TARGETS})
  set_target_properties(hover_thrust_estimator_core PROPERTIES INSTALL_RPATH "$ORIGIN")
  target_include_directories(hover_thrust_estimator_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> ${catkin_INCLUDE_DIRS})
  target_link_libraries(hover_thrust_estimator_core PUBLIC
    HoverThrustEstimator::Runtime ${catkin_LIBRARIES})
  add_executable(hover_thrust_estimator_node src/hover_thrust_estimator_main.cpp)
  add_dependencies(hover_thrust_estimator_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(hover_thrust_estimator_node PRIVATE hover_thrust_estimator_core)
  install(TARGETS hover_thrust_estimator_core hover_thrust_estimator_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(DIRECTORY include/${PROJECT_NAME}/input include/${PROJECT_NAME}/output
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
  install(FILES include/${PROJECT_NAME}/hover_thrust_estimator_node.h
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
  install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endif()

install(TARGETS hover_thrust_estimator_math EXPORT HoverThrustEstimatorTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/${PROJECT_NAME}/common include/${PROJECT_NAME}/state_machine
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})
install(FILES include/${PROJECT_NAME}/hover_thrust_estimator.h
  include/${PROJECT_NAME}/hover_thrust_estimator_runtime.h
  include/${PROJECT_NAME}/hover_thrust_output_model.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})
configure_package_config_file(cmake/HoverThrustEstimatorConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/HoverThrustEstimatorConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/HoverThrustEstimator)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/HoverThrustEstimatorConfigVersion.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(EXPORT HoverThrustEstimatorTargets NAMESPACE HoverThrustEstimator::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/HoverThrustEstimator)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/HoverThrustEstimatorConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/HoverThrustEstimatorConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/HoverThrustEstimator)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  if(BUILD_ROS AND CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    catkin_add_gtest(hover_thrust_estimator_core_test
      test/hover_thrust_estimator_test.cpp test/hover_thrust_rejection_test.cpp)
  else()
    add_executable(hover_thrust_estimator_core_test
      test/hover_thrust_estimator_test.cpp test/hover_thrust_rejection_test.cpp)
  endif()
  if(TARGET GTest::gtest)
    set(hte_gtest GTest::gtest)
  else()
    set(hte_gtest GTest::GTest)
  endif()
  target_link_libraries(hover_thrust_estimator_core_test
    HoverThrustEstimator::Runtime ${hte_gtest} pthread)
  add_test(NAME hover_thrust_estimator_core COMMAND hover_thrust_estimator_core_test)
  if(BUILD_ROS AND BUILD_NATIVE)
    if(CATKIN_ENABLE_TESTING)
      add_rostest_gtest(hover_thrust_facade_equivalence_test
        test/hover_thrust_facade_equivalence.test test/hover_thrust_facade_equivalence_test.cpp)
    else()
      add_executable(hover_thrust_facade_equivalence_test test/hover_thrust_facade_equivalence_test.cpp)
    endif()
    target_link_libraries(hover_thrust_facade_equivalence_test
      hover_thrust_estimator_core XgcRuntime::SDK ${hte_gtest} dl pthread)
    target_compile_definitions(hover_thrust_facade_equivalence_test PRIVATE
      HTE_NATIVE_MODULE_PATH="$<TARGET_FILE:hover_thrust_estimator_native>")
    add_dependencies(hover_thrust_facade_equivalence_test hover_thrust_estimator_native)
  endif()
  if(BUILD_ROS AND CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(${PROJECT_NAME}_node_params_test
      test/hover_thrust_estimator_node_params.test test/hover_thrust_estimator_node_params_test.cpp)
    if(TARGET ${PROJECT_NAME}_node_params_test)
      add_dependencies(${PROJECT_NAME}_node_params_test ${catkin_EXPORTED_TARGETS})
      target_link_libraries(${PROJECT_NAME}_node_params_test hover_thrust_estimator_core ${catkin_LIBRARIES})
    endif()
  endif()
endif()
