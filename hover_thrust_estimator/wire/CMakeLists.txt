cmake_minimum_required(VERSION 3.16)
project(HoverThrustNativeWire VERSION 1.0.0 LANGUAGES NONE)
set(CMAKE_INSTALL_INCLUDEDIR "include" CACHE PATH "Header installation directory")
set(CMAKE_INSTALL_DATADIR "share" CACHE PATH "Architecture-independent data directory")
include(CMakePackageConfigHelpers)
option(HTE_WIRE_BUILD_SMOKE "Build only the independent C11/C++11 wire layout smoke" OFF)

add_library(hover_thrust_native_wire INTERFACE)
add_library(HoverThrustNative::Wire ALIAS hover_thrust_native_wire)
set_target_properties(hover_thrust_native_wire PROPERTIES EXPORT_NAME Wire)
target_include_directories(hover_thrust_native_wire INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
install(FILES ../include/hover_thrust_estimator/native/hover_thrust_wire.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hover_thrust_estimator/native)
install(TARGETS hover_thrust_native_wire EXPORT HoverThrustNativeWireTargets)
set(wire_config_dir ${CMAKE_INSTALL_DATADIR}/cmake/HoverThrustNativeWire)
install(EXPORT HoverThrustNativeWireTargets NAMESPACE HoverThrustNative::
  DESTINATION ${wire_config_dir})
configure_package_config_file(HoverThrustNativeWireConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/HoverThrustNativeWireConfig.cmake
  INSTALL_DESTINATION ${wire_config_dir})
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/HoverThrustNativeWireConfigVersion.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion ARCH_INDEPENDENT)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/HoverThrustNativeWireConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/HoverThrustNativeWireConfigVersion.cmake
  DESTINATION ${wire_config_dir})

if(HTE_WIRE_BUILD_SMOKE)
  enable_language(C)
  enable_language(CXX)
  enable_testing()
  add_executable(hte_wire_c_smoke layout_smoke.c)
  target_compile_features(hte_wire_c_smoke PRIVATE c_std_11)
  target_link_libraries(hte_wire_c_smoke PRIVATE HoverThrustNative::Wire)
  add_executable(hte_wire_cpp_smoke layout_smoke.cpp)
  target_compile_features(hte_wire_cpp_smoke PRIVATE cxx_std_11)
  target_link_libraries(hte_wire_cpp_smoke PRIVATE HoverThrustNative::Wire)
  foreach(smoke hte_wire_c_smoke hte_wire_cpp_smoke)
    target_compile_options(${smoke} PRIVATE -Wall -Wextra -Werror -pedantic)
    add_test(NAME ${smoke} COMMAND ${smoke})
  endforeach()
endif()
