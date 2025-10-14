cmake_minimum_required(VERSION 3.8)
project(longitudinal_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pid REQUIRED)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../pid/include
)

# Add executable
add_executable(longitudinal_controller
  src/main.cpp
  src/longitudinal_controller.cpp
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Link ROS 2 dependencies
ament_target_dependencies(longitudinal_controller
  rclcpp
  std_msgs
  pid
)

# Link the pid library to your executable
target_link_libraries(longitudinal_controller pid::pid)

# Install executable
install(TARGETS longitudinal_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Optional: linting support
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
