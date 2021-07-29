cmake_minimum_required(VERSION 3.0.2)
project(msdp)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  rviz_visual_tools
  moveit_visual_tools
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  tf
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  # visp_bridge
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
  the_mainest
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
# find_package(VISP REQUIRED)


include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})


catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    the_mainest
  DEPENDS
    EIGEN3
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})


add_executable(add_object_node src/add_object_node.cpp)
target_link_libraries(add_object_node
  ${catkin_LIBRARIES}
)
add_dependencies(add_object_node ${catkin_EXPORTED_TARGETS})

add_executable(pick_and_place src/pick_and_place.cpp)
target_link_libraries(pick_and_place
  ${catkin_LIBRARIES}
)
add_dependencies(pick_and_place ${catkin_EXPORTED_TARGETS})