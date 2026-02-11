cmake_minimum_required(VERSION 3.16)
project(custom_interfaces)

# Find build dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define custom message files
set(msg_files
  "msg/RcChannels.msg",
  "msg/FlightMode.msg"
)

# Generate message code
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

# Export dependencies for downstream packages
ament_export_dependencies(
  std_msgs
  rosidl_default_runtime
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_files()
endif()

ament_package()
