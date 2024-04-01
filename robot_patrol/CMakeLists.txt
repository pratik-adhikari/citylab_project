cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

# Default to C99 for C and C++14 for C++
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDirection.srv"
  DEPENDENCIES sensor_msgs
)

# Include directories
include_directories(include)

# Declare patrol_node executable
add_executable(patrol_node src/patrol.cpp)
ament_target_dependencies(patrol_node
  rclcpp
  geometry_msgs
  sensor_msgs
)

# Declare direction_service executable
add_executable(direction_service src/direction_service.cpp)
ament_target_dependencies(direction_service
  rclcpp
  sensor_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(direction_service "${cpp_typesupport_target}")

add_executable(test_service src/test_service.cpp)
ament_target_dependencies(test_service
  rclcpp
  sensor_msgs
)
target_link_libraries(test_service "${cpp_typesupport_target}")

# Install executables
install(TARGETS
  patrol_node
  direction_service
  test_service
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Setup testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package setup
ament_package()
