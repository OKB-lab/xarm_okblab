cmake_minimum_required(VERSION 3.8)
project(okblab_api)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(xarm_msgs REQUIRED)

add_executable(xarm6_activate src/xarm6_activate.cpp)
ament_target_dependencies( xarm6_activate
  rclcpp
  xarm_msgs
)

add_executable(move_cont_cart_vel src/move_cont_cart_vel.cpp)
ament_target_dependencies(move_cont_cart_vel
  rclcpp
  tf2
  xarm_msgs
)

add_executable(move_cont_cart_pos src/move_cont_cart_pos.cpp)
ament_target_dependencies(move_cont_cart_pos
  rclcpp
  xarm_msgs
)

install(TARGETS
  xarm6_activate
  move_cont_cart_vel
  move_cont_cart_pos
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
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
