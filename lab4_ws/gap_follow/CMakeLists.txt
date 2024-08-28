cmake_minimum_required(VERSION 3.5)
project(gap_follow)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)  # Ensure this line is present
find_package(rosidl_default_generators REQUIRED)  # Ensure this line is present

# Include directories
include_directories(include)

# Create Cpp executable
add_executable(reactive_node src/reactive_node.cpp)

# Link dependencies
ament_target_dependencies(reactive_node 
  rclcpp 
  geometry_msgs 
  ackermann_msgs 
  nav_msgs 
  sensor_msgs 
  std_msgs 
  visualization_msgs  # Ensure this line is present
)

# Install Cpp executables
install(TARGETS
  reactive_node
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/reactive_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

