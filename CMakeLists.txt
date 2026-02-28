cmake_minimum_required(VERSION 3.8)
project(trajectory_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

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

  ament_add_gtest(test_path_smoother test/test_path_smoother.cpp)
  target_link_libraries(test_path_smoother trajectory_tracking_math)

  ament_add_gtest(test_trajectory_generator test/test_trajectory_generator.cpp)
  target_link_libraries(test_trajectory_generator trajectory_tracking_math)

  ament_add_gtest(test_pure_pursuit test/test_pure_pursuit.cpp)
  target_link_libraries(test_pure_pursuit trajectory_tracking_math)
endif()

# Core Math Libraries
add_library(trajectory_tracking_math
  src/b_spline_smoother.cpp
  src/trajectory_generator.cpp
  src/pure_pursuit.cpp
)
target_include_directories(trajectory_tracking_math PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

add_executable(trajectory_tracker_node src/trajectory_tracker_node.cpp)
target_link_libraries(trajectory_tracker_node trajectory_tracking_math)
ament_target_dependencies(trajectory_tracker_node rclcpp nav_msgs geometry_msgs sensor_msgs tf2)

install(
  TARGETS trajectory_tracking_math trajectory_tracker_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp nav_msgs geometry_msgs sensor_msgs tf2)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
