cmake_minimum_required(VERSION 3.8)
project(bmx160_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

# Include directories
include_directories(include)

# Create library
add_library(bmx160_lib src/bmx160.cpp)

# Create executable
add_executable(bmx160_node src/bmx160_node.cpp)
target_link_libraries(bmx160_node bmx160_lib)
ament_target_dependencies(bmx160_node rclcpp sensor_msgs rcl_interfaces)

# Install C++ targets
install(TARGETS
  bmx160_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Install Python node and module
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  bmx160_ros/mag_cal_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
