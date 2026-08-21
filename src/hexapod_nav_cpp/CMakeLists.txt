cmake_minimum_required(VERSION 3.16)
project(hexapod_nav_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# Gait controller node
add_executable(gait_controller_node src/gait_controller_node.cpp)
ament_target_dependencies(gait_controller_node
  rclcpp std_msgs geometry_msgs nav_msgs grid_map_msgs visualization_msgs
)

# Foothold planner node
add_executable(foothold_planner_node src/foothold_planner_node.cpp)
ament_target_dependencies(foothold_planner_node
  rclcpp std_msgs geometry_msgs nav_msgs grid_map_msgs visualization_msgs
)

# Terrain cost node
add_executable(terrain_cost_node src/terrain_cost_node.cpp)
ament_target_dependencies(terrain_cost_node
  rclcpp std_msgs grid_map_msgs
)

# Octomap terrain node
add_executable(octomap_terrain_node src/octomap_terrain_node.cpp)
ament_target_dependencies(octomap_terrain_node
  rclcpp std_msgs sensor_msgs grid_map_msgs tf2_ros tf2_geometry_msgs
)

install(TARGETS
  gait_controller_node
  foothold_planner_node
  terrain_cost_node
  octomap_terrain_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Install launch, config, and rviz files from the source tree
install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Symlink configs from hexapod_nav (shared config)
install(DIRECTORY
  ../hexapod_nav/config
  DESTINATION share/${PROJECT_NAME}
  EXCLUDE_FROM_ALL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
