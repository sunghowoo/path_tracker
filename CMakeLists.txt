cmake_minimum_required(VERSION 3.5)
project(path_tracker)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependenciesa
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
include_directories(include)

# Build
add_executable(path_generator
  src/path_generator_node.cpp
)

ament_target_dependencies(path_generator
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
)

add_executable(pure_pursuit
  src/pure_pursuit_node.cpp
)

ament_target_dependencies(pure_pursuit
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  visualization_msgs
  std_srvs
)

# Install
install(TARGETS
  path_generator
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  pure_pursuit
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Macro for ament package
ament_package()