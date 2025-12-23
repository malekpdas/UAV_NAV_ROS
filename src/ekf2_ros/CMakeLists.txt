cmake_minimum_required(VERSION 3.8)
project(ekf2_ros)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  extern
  ${Eigen3_INCLUDE_DIRS}
)

# Fusion Library (C files)
set(FUSION_SRC
  extern/Fusion/FusionAhrs.c
  extern/Fusion/FusionBias.c
  extern/Fusion/FusionCompass.c
)

# Create Node executable
add_executable(ekf_node
  src/ekf_node.cpp
  ${FUSION_SRC}
)

target_link_libraries(ekf_node
  ${Eigen3_LIBRARIES}
)

ament_target_dependencies(ekf_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Install targets
install(TARGETS
  ekf_node
  DESTINATION lib/${PROJECT_NAME}
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
