cmake_minimum_required(VERSION 3.8)
project(demo_cpp_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(srv_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(turtle_service src/turtle_service.cpp)
ament_target_dependencies(turtle_service rclcpp geometry_msgs turtlesim_msgs srv_interfaces)

add_executable(turtle_client src/turtle_client.cpp)
ament_target_dependencies(turtle_client rclcpp geometry_msgs turtlesim_msgs srv_interfaces)

install(
TARGETS turtle_service
TARGETS turtle_client
DESTINATION lib/${PROJECT_NAME}
)

install(
DIRECTORY launch
DESTINATION share/${PROJECT_NAME}
)

ament_package()
