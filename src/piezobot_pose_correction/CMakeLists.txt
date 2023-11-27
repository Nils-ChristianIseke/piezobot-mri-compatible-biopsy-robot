cmake_minimum_required(VERSION 3.8)
project(piezobot_pose_correction)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(trajectory_msgs REQUIRED)

# add_executable(target_publisher src/target_publisher.cpp)

# target_include_directories(target_publisher PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# target_compile_features(target_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ament_target_dependencies(
#   target_publisher
#   "moveit_ros_planning_interface"
#   "rclcpp"
#   "visualization_msgs"
#   "geometry_msgs"
#   "trajectory_msgs"
#   "moveit_core"
#   "Eigen3"
# )

# install(TARGETS target_publisher
#   DESTINATION lib/${PROJECT_NAME})

add_executable(pose_correction src/pose_correction.cpp)

target_include_directories(pose_correction PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(pose_correction PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  pose_correction
  "moveit_ros_planning_interface"
  "rclcpp"
  "visualization_msgs"
  "geometry_msgs"
  "trajectory_msgs"
  "Eigen3"
)
install(TARGETS pose_correction
  DESTINATION lib/${PROJECT_NAME})

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
