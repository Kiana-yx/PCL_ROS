cmake_minimum_required(VERSION 3.0.2)
project(tim_pkg)

add_compile_options(-std=c++14)


find_package(catkin REQUIRED COMPONENTS
pcl_ros
roscpp
sensor_msgs
)

find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(tim_base_node src/tim_base_node.cpp src/tim_base_core.cpp)


target_link_libraries(tim_base_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
