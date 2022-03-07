cmake_minimum_required(VERSION 3.0.2)
project(map_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
)


find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES map_pkg
 CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs pcl_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(grid_node src/grid_node.cpp src/grid_core.cpp)
target_link_libraries(grid_node 
${catkin_LIBRARIES}
${PCL_LIBRARIES}
)
