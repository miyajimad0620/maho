cmake_minimum_required(VERSION 3.14)

project(maho LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(
  maho
  src/collision_detector.cpp
  src/evaluation_function.cpp
  src/expander.cpp
  src/goal_velocity_cost.cpp
  src/kinematics.cpp
  src/maho.cpp
  src/optimizer.cpp
  src/selector.cpp
)

target_include_directories(
  maho
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(maho PUBLIC cxx_std_17)

add_executable(maho_example example.cpp)
target_link_libraries(maho_example PRIVATE maho)

include(CTest)

if(BUILD_TESTING)
  add_executable(collision_detector_test test/collision_detector_test.cpp)
  target_link_libraries(collision_detector_test PRIVATE maho)
  add_test(NAME collision_detector_test COMMAND collision_detector_test)

  add_executable(evaluation_function_test test/evaluation_function_test.cpp)
  target_link_libraries(evaluation_function_test PRIVATE maho)
  add_test(NAME evaluation_function_test COMMAND evaluation_function_test)

  add_executable(expander_test test/expander_test.cpp)
  target_link_libraries(expander_test PRIVATE maho)
  add_test(NAME expander_test COMMAND expander_test)

  add_executable(selector_test test/selector_test.cpp)
  target_link_libraries(selector_test PRIVATE maho)
  add_test(NAME selector_test COMMAND selector_test)

  add_executable(optimizer_test test/optimizer_test.cpp)
  target_link_libraries(optimizer_test PRIVATE maho)
  add_test(NAME optimizer_test COMMAND optimizer_test)

  add_executable(maho_test test/maho_test.cpp)
  target_link_libraries(maho_test PRIVATE maho)
  add_test(NAME maho_test COMMAND maho_test)
endif()
