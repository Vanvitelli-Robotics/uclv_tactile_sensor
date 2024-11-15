cmake_minimum_required(VERSION 3.8)
project(uclv_tactile_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Migrating codes
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(uclv_tactile_common REQUIRED)
find_package(serial REQUIRED)
# find_package(sun_systems_lib REQUIRED)
# find_package(ros_toon)
# find_package(sun_learning_lib REQUIRED)
# find_package(sun_ros_msgs REQUIRED)

include_directories(
  include
  ${sun_systems_lib_INCLUDE_DIRS}
)

add_executable(read_tactile_serial src/read_tactile_serial_node.cpp)
ament_target_dependencies(read_tactile_serial rclcpp uclv_tactile_common serial)

# add_executable(tactile_logic_filter src/tactile_logic_filter_node.cpp)
# ament_target_dependencies(tactile_logic_filter rclcpp uclv_tactile_common)

# add_executable(filter_voltages src/filter_tactile_node.cpp)
# ament_target_dependencies(filter_voltages rclcpp uclv_tactile_common sun_systems_lib ros_toon)

add_executable(remove_bias src/remove_bias_node.cpp)
ament_target_dependencies(remove_bias rclcpp rclcpp_action uclv_tactile_common)

install(TARGETS
  read_tactile_serial
  DESTINATION lib/${PROJECT_NAME}
)

# install(TARGETS
#   tactile_logic_filter
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(TARGETS
#   filter_voltages
#   DESTINATION lib/${PROJECT_NAME}
# )

install(TARGETS
  remove_bias
  DESTINATION lib/${PROJECT_NAME}
)
# End migrating codes


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

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()