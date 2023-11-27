cmake_minimum_required(VERSION 3.8)
project(piezobot_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# add_executable(servo_keyboard_input src/servo_keyboard_input.cpp)
# target_include_directories(servo_keyboard_input PUBLIC include)
# ament_target_dependencies(servo_keyboard_input  ${THIS_PACKAGE_INCLUDE_DEPENDS})
find_package(rclcpp REQUIRED)
find_package(moveit_servo REQUIRED)

add_executable(servo_cpp_interface_deo src/servo_cpp_interface_deo.cpp)
target_include_directories(servo_cpp_interface_deo PUBLIC include)
ament_target_dependencies(servo_cpp_interface_deo  ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS
    servo_cpp_interface_deo
  DESTINATION
    lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
ament_target_dependencies(
  servo_cpp_interface_deo PUBLIC
  rclcpp
  moveit_servo
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
