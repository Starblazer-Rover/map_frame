cmake_minimum_required(VERSION 3.5)
project(map_frame)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CostMap.srv"
)

ament_export_dependencies(rosidl_default_runtime)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install other Python files
install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

add_service_files(
  FILES
  CostMap.srv
  TargetPoint.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
)

ament_package()
