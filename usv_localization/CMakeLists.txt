cmake_minimum_required(VERSION 3.5)
project(usv_localization)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(pose_add_covariance
  src/pose_add_covariance.cpp
)
ament_target_dependencies(pose_add_covariance
  rclcpp
  geometry_msgs
  std_msgs
)
install(TARGETS
  pose_add_covariance
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(twist_add_covariance
  src/twist_add_covariance.cpp
)
ament_target_dependencies(twist_add_covariance
  rclcpp
  geometry_msgs
  std_msgs
)
install(TARGETS
twist_add_covariance
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(dead_reckoning
  src/dead_reckoning.cpp
)
ament_target_dependencies(dead_reckoning
  rclcpp
  geometry_msgs
  nav_msgs
)
install(TARGETS
dead_reckoning
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
