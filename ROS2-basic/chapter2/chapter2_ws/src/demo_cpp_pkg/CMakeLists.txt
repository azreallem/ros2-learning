cmake_minimum_required(VERSION 3.8)
project(demo_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
include_directories(include)

add_executable(cpp_node src/cpp_node.cpp)
ament_target_dependencies(cpp_node rclcpp)
install(TARGETS
cpp_node
DESTINATION lib/${PROJECT_NAME}
)

add_executable(person_node src/person_node.cpp)
ament_target_dependencies(person_node rclcpp)
install(TARGETS
person_node
DESTINATION lib/${PROJECT_NAME}
)

add_executable(learn_auto src/learn_auto.cpp)
install(TARGETS
learn_auto
DESTINATION lib/${PROJECT_NAME}
)

add_executable(learn_shared_ptr src/learn_shared_ptr.cpp)
install(TARGETS
learn_shared_ptr
DESTINATION lib/${PROJECT_NAME}
)

add_executable(learn_lambda src/learn_lambda.cpp)
install(TARGETS
learn_lambda
DESTINATION lib/${PROJECT_NAME}
)

add_executable(learn_function src/learn_function.cpp)
install(TARGETS
learn_function
DESTINATION lib/${PROJECT_NAME}
)

add_executable(learn_thread src/learn_thread.cpp)
install(TARGETS
learn_thread
DESTINATION lib/${PROJECT_NAME}
)


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

ament_package()
