cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(ball_and_pail_sim)

# Find the Gazebo_Transport library
find_package(gz-transport13 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

include_directories(${CMAKE_BINARY_DIR})

if (EXISTS "${CMAKE_SOURCE_DIR}/ball_pail.cc")
  add_executable(ball_pail ball_pail.cc)
  target_link_libraries(ball_pail gz-transport${GZ_TRANSPORT_VER}::core)
endif()