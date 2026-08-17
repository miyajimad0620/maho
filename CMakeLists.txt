cmake_minimum_required(VERSION 3.14)

project(maho LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(
  maho
  src/evaluator.cpp
  src/expander.cpp
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
  add_executable(evaluator_test test/evaluator_test.cpp)
  target_link_libraries(evaluator_test PRIVATE maho)
  add_test(NAME evaluator_test COMMAND evaluator_test)

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
