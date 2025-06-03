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
find_package(moveit_core REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(nullspace_explorer_lib src/nullspace_exploration.cpp)

add_executable(draw_function_whiteboard src/draw_function_whiteboard.cpp)

target_link_libraries(draw_function_whiteboard nullspace_explorer_lib)


target_include_directories(draw_function_whiteboard PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  draw_function_whiteboard
  moveit_ros_planning_interface
  rclcpp
  moveit_visual_tools
  tf2_geometry_msgs
  tf2
  tf2_ros 
  tf2_eigen
  geometry_msgs
)

add_executable(draw_function_air src/draw_function_air.cpp)
target_include_directories(draw_function_air PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  draw_function_air
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

add_executable(frame_broadcaster src/frame_broadcaster.cpp)
ament_target_dependencies(
    frame_broadcaster
    geometry_msgs
    rclcpp
    tf2_geometry_msgs
    tf2
    tf2_ros 
    tf2_eigen
)

target_include_directories(nullspace_explorer_lib PUBLIC include ${rclcpp_INCLUDE_DIRS})
target_include_directories(nullspace_explorer_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
    nullspace_explorer_lib
    rclcpp 
    moveit_core
    moveit_ros_planning_interface
    control_msgs
)

install(TARGETS 
  draw_function_whiteboard
  draw_function_air
  fixed_frame_tf2_broadcaster
  frame_broadcaster
  nullspace_explorer_lib
  DESTINATION lib/${PROJECT_NAME})

  # Install launch files.
install(DIRECTORY
  launch
  models
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
