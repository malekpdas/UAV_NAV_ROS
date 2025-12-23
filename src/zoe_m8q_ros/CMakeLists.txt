cmake_minimum_required(VERSION 3.8)
project(zoe_m8q_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Create library
add_library(zoe_m8q_lib src/zoe_m8q.cpp)

# Create executable
add_executable(zoe_m8q_node src/zoe_m8q_node.cpp)
target_link_libraries(zoe_m8q_node zoe_m8q_lib)
ament_target_dependencies(zoe_m8q_node rclcpp sensor_msgs geometry_msgs)

# Install targets
install(TARGETS
  zoe_m8q_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch and config
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
