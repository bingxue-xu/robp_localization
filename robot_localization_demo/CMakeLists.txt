cmake_minimum_required(VERSION 3.8)
project(robot_localization_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)
find_package(robot_localization REQUIRED)

# include directories
include_directories(include)

# create libraries
add_library(positioning_system src/sensors/positioning_system.cpp)
add_library(odometry src/sensors/odometry.cpp)
ament_target_dependencies(positioning_system rclcpp geometry_msgs turtlesim tf2 tf2_geometry_msgs tf2_ros)
ament_target_dependencies(odometry rclcpp geometry_msgs turtlesim tf2 tf2_geometry_msgs tf2_ros)
# create executables
add_executable(positioning_system_node src/sensors/positioning_system_node.cpp)
add_executable(odometry_node src/sensors/odometry_node.cpp)
add_executable(transformation_visualization_node src/transforms/transformation_visualization_node.cpp)
# link libraries to executables
target_link_libraries(positioning_system_node positioning_system)
target_link_libraries(odometry_node odometry)

#link executables to dependencies
ament_target_dependencies(positioning_system_node rclcpp geometry_msgs turtlesim tf2 tf2_geometry_msgs tf2_ros)
ament_target_dependencies(odometry_node rclcpp geometry_msgs turtlesim tf2 tf2_geometry_msgs tf2_ros)
ament_target_dependencies(transformation_visualization_node rclcpp geometry_msgs turtlesim tf2 tf2_geometry_msgs tf2_ros)

# install executables
install(TARGETS
  positioning_system_node
  odometry_node
  transformation_visualization_node
  DESTINATION lib/${PROJECT_NAME})

# install header files
install(DIRECTORY include/
  DESTINATION include)

# install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
