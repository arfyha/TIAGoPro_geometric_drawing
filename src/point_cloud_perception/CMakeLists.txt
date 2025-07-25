cmake_minimum_required(VERSION 3.8)
project(point_cloud_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include  
  ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(pre_process src/pre_process.cpp)
target_include_directories(pre_process PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(pre_process rclcpp tf2_ros tf2 tf2_eigen sensor_msgs visualization_msgs geometry_msgs pcl_ros)

target_link_libraries(
  pre_process
  ${PCL_LIBRARIES}
)

add_executable(euclidean_cluster src/euclidean_cluster.cpp)
target_include_directories(euclidean_cluster PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(euclidean_cluster rclcpp tf2_ros tf2 tf2_eigen sensor_msgs visualization_msgs geometry_msgs pcl_ros)

target_link_libraries(
  euclidean_cluster
  ${PCL_LIBRARIES}
)

add_executable(plane_segmentation src/plane_segmentation.cpp)
target_include_directories(plane_segmentation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(plane_segmentation rclcpp tf2_ros tf2 tf2_eigen sensor_msgs visualization_msgs geometry_msgs pcl_ros)

target_link_libraries(
  plane_segmentation
  ${PCL_LIBRARIES}
)


install(TARGETS
  pre_process
  euclidean_cluster
  plane_segmentation
  DESTINATION lib/${PROJECT_NAME})

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
