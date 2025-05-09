cmake_minimum_required(VERSION 3.8)
project(draw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)

add_executable(draw_circle_air src/draw_circle_air.cpp)
add_executable(draw_circle_whiteboard src/draw_circle_whiteboard.cpp)
target_include_directories(draw_circle_air PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(draw_circle_whiteboard PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(draw_circle_air PUBLIC c_std_99 cxx_std_17)
target_compile_features(draw_circle_whiteboard PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  draw_circle_air
  moveit_ros_planning_interface
  rclcpp
  moveit_visual_tools
  tf2_geometry_msgs
  tf2
  tf2_ros 
  tf2_eigen
)
ament_target_dependencies(
  draw_circle_whiteboard
  moveit_ros_planning_interface
  rclcpp
  moveit_visual_tools
  tf2_geometry_msgs
  tf2
)

add_executable(fixed_frame_tf2_broadcaster src/fixed_frame_tf2_broadcaster.cpp)
ament_target_dependencies(
    fixed_frame_tf2_broadcaster
    geometry_msgs
    rclcpp
    tf2_geometry_msgs
    tf2
    tf2_ros 
    tf2_eigen
)

install(TARGETS 
  draw_circle_air
  draw_circle_whiteboard
  fixed_frame_tf2_broadcaster
  DESTINATION lib/${PROJECT_NAME})

  # Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

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
