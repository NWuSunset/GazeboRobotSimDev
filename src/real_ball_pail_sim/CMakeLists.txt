cmake_minimum_required(VERSION 3.8)
project(real_ball_pail_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ball_publisher src/ball_publisher.cpp)
ament_target_dependencies(ball_publisher rclcpp geometry_msgs ros_gz_interfaces std_msgs)

install(TARGETS ball_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config models worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
